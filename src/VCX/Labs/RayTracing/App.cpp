#include "Assets/bundled.h"
#include "Labs/RayTracing/App.h"

namespace VCX::Labs::Rendering {
    using namespace Assets;

    App::App() :
        _ui(Labs::Common::UIOptions { }),
        _caseRayTracing({ ExampleScene::Floor, ExampleScene::CornellBox, ExampleScene::WhiteOak, ExampleScene::Sponza,ExampleScene::CornellBoxSphere,ExampleScene::SportsCar,ExampleScene::Sibenik }) ,
        _casePathTracing({ ExampleScene::Floor, ExampleScene::CornellBox, ExampleScene::CornellBoxArea, ExampleScene::CornellBoxSphere }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
