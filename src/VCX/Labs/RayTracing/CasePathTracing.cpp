#include "Labs/RayTracing/CasePathTracing.h"
#include <future>
namespace VCX::Labs::Rendering {

    CasePathTracing::CasePathTracing(std::initializer_list<Assets::ExampleScene> && scenes):
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(4),
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }) {
        _cameraManager.AutoRotate = false;
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1, 1, 1));
        sum.resize(_buffer.GetSizeX());
        for (int i = 0; i < _buffer.GetSizeX(); i++)
            sum[i].resize(_buffer.GetSizeY());
        for (int i = 0; i < _buffer.GetSizeX(); i++)
            for (int j = 0; j < _buffer.GetSizeY(); j++)
                sum[i][j] = glm::vec3(0.0f);
    }

    CasePathTracing::~CasePathTracing() {
        _stopFlag = true;
        if (_task.joinable()) _task.join();
    }

    void CasePathTracing::OnSetupPropsUI() {
        if (ImGui::BeginCombo("Scene", GetSceneName(_sceneIdx))) {
            for (std::size_t i = 0; i < _scenes.size(); ++i) {
                bool selected = i == _sceneIdx;
                if (ImGui::Selectable(GetSceneName(i), selected)) {
                    if (! selected) {
                        _sceneIdx   = i;
                        _sceneDirty = true;
                        _treeDirty  = true;
                        _resetDirty = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        if (ImGui::Button("Reset Scene")) _resetDirty = true;
        ImGui::SameLine();
        if (_task.joinable()) {
            if (ImGui::Button("Stop Rendering")) {
                _stopFlag = true;
                if (_task.joinable()) _task.join();
            }
        } else if (ImGui::Button("Start Rendering")) _stopFlag = false;
        ImGui::ProgressBar(float(_pixelIndex) / (_buffer.GetSizeX() * _buffer.GetSizeY()));
        Common::ImGuiHelper::SaveImage(_texture, GetBufferSize(), true);
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            _resetDirty |= ImGui::SliderFloat("Possibility of Russian Roulette", &_PRR, 0.1, 0.9);
            _resetDirty |= ImGui::SliderInt("Samples Per Pixel", &_max_SPP, 20, 10000);
            _resetDirty |= ImGui::Checkbox("Shadow Ray", &_enableShadow);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Control")) {
            ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CasePathTracing::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_resetDirty) {
            _stopFlag = true;
            if (_task.joinable()) _task.join();
            _pixelIndex = 0;
            _resizable  = true;
            _resetDirty = false;
        }
        if (_sceneDirty) {
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
            _sceneDirty = false;
        }
        if (_resizable) {
            _frame.Resize(desiredSize);
            _cameraManager.Update(_sceneObject.Camera);
            _program.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            _program.GetUniforms().SetByName("u_View", _sceneObject.Camera.GetViewMatrix());

            gl_using(_frame);

            glEnable(GL_DEPTH_TEST);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            for (auto const & model : _sceneObject.OpaqueModels)
                model.Mesh.Draw({ _program.Use() });
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_DEPTH_TEST);
        }
        if (! _stopFlag && ! _task.joinable()) {
            if (_pixelIndex == 0) {
                _resizable = false;
                _buffer    = _frame.GetColorAttachment().Download<Engine::Formats::RGB8>();
            }
            assert(_task.joinable() == false);
            std::mutex sumMutex;
            _task = std::thread([&]() {
                thread_local std::random_device rd;
                thread_local std::mt19937       gen(rd());
                thread_local std::uniform_real_distribution<float> dist_real(0.0f, 1.0f);

                auto const width  = _buffer.GetSizeX();
                auto const height = _buffer.GetSizeY();
                if (_pixelIndex == 0 && _treeDirty) {
                    Engine::Scene const & scene = GetScene(_sceneIdx);
                    _intersector.InitScene(&scene);
                    _treeDirty = false;
                }
                // Render into tex.

                _pixelIndex = 0;
                while (_pixelIndex < std::size_t(width) * height) {
                    glm::vec3 tot(0.0f);
                    int          i    = _pixelIndex % width;
                    int          j    = _pixelIndex / width;
                    for (int spp = 0; spp < _max_SPP; spp++) {
                        float        step = 1.0f;
                        float        di = dist_real(gen), dj = dist_real(gen);
                        //float        di = 0.5, dj = 0.5;
                        auto const & camera    = _sceneObject.Camera;
                        glm::vec3    lookDir   = glm::normalize(camera.Target - camera.Eye);
                        glm::vec3    rightDir  = glm::normalize(glm::cross(lookDir, camera.Up));
                        glm::vec3    upDir     = glm::normalize(glm::cross(rightDir, lookDir));
                        float const  aspect    = width * 1.f / height;
                        float const  fovFactor = std::tan(glm::radians(camera.Fovy) / 2);
                        lookDir += fovFactor * (2.0f * (j + dj) / height - 1.0f) * upDir;
                        lookDir += fovFactor * aspect * (2.0f * (i + di) / width - 1.0f) * rightDir;
                        Ray       initialRay(camera.Eye, glm::normalize(lookDir));
                        glm::vec3 res = PathTrace(_intersector, initialRay, _PRR, _enableShadow,gen);
                        assert(i < width && j < height);
                        tot += glm::pow(res, glm::vec3(1.0 / 2.2));
                        
                    }
                    _buffer.At(i, j) = tot / (float)(_max_SPP + 1);
                    ++_pixelIndex;
                    if (_stopFlag) return;
                }
            });
        }
        if (! _resizable) {
            if (! _stopFlag) _texture.Update(_buffer);
            if (_task.joinable() && _pixelIndex == _buffer.GetSizeX() * _buffer.GetSizeY()) {
                _stopFlag = true;
                _task.join();
            }
        }
        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _resizable ? _frame.GetColorAttachment() : _texture,
            .ImageSize = _resizable ? desiredSize : GetBufferSize(),
        };
    }

    void CasePathTracing::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (_resizable) {
            _cameraManager.ProcessInput(_sceneObject.Camera, pos);
        } else {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
                ImGui::SetScrollX(window, window->Scroll.x - delta.x);
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
                ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        }
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_resizable ? _frame.GetColorAttachment() : _texture, GetBufferSize(), pos, true);
    }

} // namespace VCX::Labs::Rendering
