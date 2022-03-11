#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"

namespace Falcor
{
    enum class SceneName
    {
        SimpleScene, SanMiguel, BistroExterior, BistroInterior,
        Sponza, 
    };

    struct AreaLightParams
    {
        SceneName name;
        float3 mLightPos;
        float3 mLightUp;
        float3 mRotation;
        float mLightSize;
        float mFovY;
        float mLightNearPlane;
        float mLightFarPlane;

        AreaLightParams(SceneName name) : name(name), mLightFarPlane(1e5), mRotation(float3(0.0f))
        {
            if (name == SceneName::SanMiguel)
            {
                mLightUp = float3(0.0f, 0.0f, -1.0f);
                mLightPos = float3(14.5f, 50.0f, 15.0f);
                mRotation = float3(100.0f, 0.0f, 0.0f);
                mLightSize = 10.0f;
                mFovY = 40.0f;
                mLightNearPlane = mLightPos.y - 20.0f;
            }
            else if (name == SceneName::SimpleScene)
            {
                mLightUp = float3(0.0f, 1.0f, 0.0f);
                mLightPos = float3(0.0f, 2.0f, -10.0f);
                mRotation = float3(10.0f, 0.0f, 0.0f);
                mLightSize = 1.5f;
                mFovY = 55.0;
                mLightNearPlane = 8.0f;
            }
            else if (name == SceneName::BistroExterior)
            {
                mLightUp = float3(0.0f, 1.0f, 0.0f);
                mLightPos = float3(-10.0f, 70.0f, -50.0f);
                mRotation = float3(40.0f, 0.0f, 0.0f);
                mLightSize = 15.0f;
                mFovY = 90.0;
                mLightNearPlane = 40.0f;
            }
            else if (name == SceneName::BistroInterior)
            {
                mLightUp = float3(0, 1, 0);
                mLightPos = float3(4.955003, 2.819287, 6.114000);
                mRotation = float3(160.0f, -10.0f, 0.0f);
                mLightSize = 2.0f;
                mFovY = 125.0;
                mLightNearPlane = 1.5f;
            }
            else if (name == SceneName::Sponza)
            {
                mLightUp = float3(0.0f, 1.0f, 0.0f);
                mLightPos = float3(0.0f, 20.0f, 0.0f);
                mRotation = float3(90.0f, 0.0f, 0.0f);
                mLightSize = 15.0f;
                mFovY = 90.0;
                mLightNearPlane = mLightPos.y - 10.0f;
            }
        }

        void setSceneSettings(const Scene::SharedPtr& pScene)
        {
            if (name == SceneName::BistroExterior)
            {
                pScene->setIsAnimated(false);

                pScene->selectCamera(1);
                pScene->getCamera()->setIsAnimated(false);

                auto settings = pScene->getRenderSettings();
                settings.useEnvLight = false;
                pScene->setRenderSettings(settings);

                auto& lights = pScene->getLights();
                const_cast<std::vector<Light::SharedPtr>&>(lights).erase(lights.begin());
            }
            else if (name == SceneName::BistroInterior)
            {
                pScene->setIsAnimated(false);

                pScene->selectCamera(1);
                pScene->getCamera()->setIsAnimated(false);

                auto settings = pScene->getRenderSettings();
                settings.useEnvLight = false;
                pScene->setRenderSettings(settings);

                auto& lights = pScene->getLights();
                const_cast<std::vector<Light::SharedPtr>&>(lights).erase(lights.begin(), lights.begin() + 4);
            }
        }

        void updateSceneAreaLight(const Scene::SharedPtr& pScene)
        {
            auto pLight = pScene->getLightByName("Area Light").get();
            if (!pLight) return;

            auto translation = glm::translate(glm::mat4(1.0f), mLightPos);
            auto rotationX = glm::rotate(glm::mat4(1.0f), glm::radians(mRotation.x), float3(1.0f, 0.0f, 0.0f));
            auto rotationY = glm::rotate(glm::mat4(1.0f), glm::radians(mRotation.y), float3(0.0f, 1.0f, 0.0f));
            auto rotationZ = glm::rotate(glm::mat4(1.0f), glm::radians(mRotation.z), float3(0.0f, 0.0f, 1.0f));

            auto pAreaLight = dynamic_cast<AnalyticAreaLight*>(pLight);
            auto model = translation * rotationY * rotationZ * rotationX;
            pAreaLight->setScaling(float3(mLightSize / 2.0f));
            pAreaLight->setTransformMatrix(model);
        }
        
    };

    void setSceneMesh()
    {

    }
}
