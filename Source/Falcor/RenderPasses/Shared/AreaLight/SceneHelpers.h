#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"

namespace Falcor
{
    enum class SceneName
    {
        TestScene, SanMiguel
    };

    void setSceneAreaLight(const Scene::SharedPtr& pScene, SceneName name, float3 lightPos, float lightSize, float4 rot)
    {
        // Area light normal is (0, 0, 1)
        if (name == SceneName::TestScene)
        {
            auto pLight = pScene->getLightByName("Area Light").get();
            if (!pLight) return;

            auto translation = glm::translate(glm::mat4(1.0f), lightPos);
            auto rotation = glm::rotate(glm::mat4(1.0f), glm::radians(rot.x), float3(rot.yzw));
            auto model = translation * rotation;
            auto pRectLight = dynamic_cast<RectLight*>(pLight);
            pRectLight->setScaling(float3(lightSize / 2.0f));
            pRectLight->setTransformMatrix(model);
        }
        else if (name == SceneName::SanMiguel)
        {
            auto pLight = pScene->getLightByName("Area Light").get();
            if (!pLight) return;

            auto translation = glm::translate(glm::mat4(1.0f), lightPos);
            auto rotation = glm::rotate(glm::mat4(1.0f), glm::radians(rot.x), float3(rot.yzw));

            auto lightType = pLight->getType();
            auto pAreaLight = dynamic_cast<AnalyticAreaLight*>(pLight);
            auto model = lightType == LightType::Rect ? translation * rotation : translation;
            pAreaLight->setScaling(float3(lightSize / 2.0f));
            pAreaLight->setTransformMatrix(model);
        }
    }

    void setSceneMesh()
    {

    }
}
