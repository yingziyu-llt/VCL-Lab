#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/RayTracing/CaseRayTracing.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Rendering {
    class App : public Engine::IApp {
    private:
        Common::UI         _ui;
        CaseRayTracing     _caseRayTracing;

        std::size_t        _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseRayTracing,
        };

    public:
        App();

        void OnFrame() override;
    };
}
