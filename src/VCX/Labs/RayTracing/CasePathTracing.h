#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Labs/RayTracing/Content.h"
#include "Labs/RayTracing/SceneObject.h"
#include "Labs/RayTracing/PathTracing.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include <random>

namespace VCX::Labs::Rendering {

    class CasePathTracing : public Common::ICase {
    public:
        CasePathTracing(std::initializer_list<Assets::ExampleScene> && scenes);
        ~CasePathTracing();

        virtual std::string_view const GetName() override { return "Path Tracing"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        std::vector<Assets::ExampleScene> const _scenes;
        Engine::GL::UniqueProgram               _program;
        Engine::GL::UniqueRenderFrame           _frame;
        SceneObject                             _sceneObject;
        Common::OrbitCameraManager              _cameraManager;

        Engine::GL::UniqueTexture2D _texture;
        RayIntersector              _intersector;

        std::size_t                             _sceneIdx { 0 };
        bool                                    _enableZoom { true };
        bool                                    _enableShadow { true };
        int                                     _max_SPP { 1000 };
        float _PRR {0.8f};
        std::size_t                             _pixelIndex { 0 };
        bool                                    _stopFlag { true };
        bool                                    _sceneDirty { true };
        bool                                    _treeDirty { true };
        bool                                    _resetDirty { true };
        Common::ImageRGB                        _buffer;
        bool                                    _resizable { true };
        std::vector<std::vector<glm::vec3>> sum;

        std::thread _task;

        auto GetBufferSize() const { return std::pair(std::uint32_t(_buffer.GetSizeX()), std::uint32_t(_buffer.GetSizeY())); }

        char const *          GetSceneName(std::size_t const i) const { return Content::SceneNames[std::size_t(_scenes[i])].c_str(); }
        Engine::Scene const & GetScene(std::size_t const i) const { return Content::Scenes[std::size_t(_scenes[i])]; }
    };
} // namespace VCX::Labs::Rendering
