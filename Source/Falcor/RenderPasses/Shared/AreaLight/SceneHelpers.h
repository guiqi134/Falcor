#pragma once
#include "Falcor.h"
//#include "FalcorExperimental.h"

namespace Falcor
{
    enum class SceneName
    {
        SimpleScene, SanMiguel, BistroExterior, BistroInterior,
        OnePlaneScene, ThreePlaneScene, Bicycle, PalmTrees,
    };

    struct AreaLightParams
    {
        SceneName name;
        float3 mLightPos;
        float3 mLightUp;
        float3 mRotation;
        float2 mLightSize; 
        float mFovY;
        float mLightNearPlane;
        float mLightFarPlane;
        float mDepthBias;
        float4x4 mObjectToWorld;

        AreaLightParams(SceneName name) : name(name), mLightFarPlane(1e5), mRotation(float3(0.0f)), mDepthBias(0.001f)
        {
            if (name == SceneName::SanMiguel)
            {
                mLightUp = float3(0.0f, 0.0f, -1.0f);
                mLightPos = float3(14.5f, 50.0f, 15.0f);
                mRotation = float3(107.0f, 0.0f, 0.0f);
                mLightSize = float2(3.0f);
                mFovY = 35.0f;
                mLightNearPlane = mLightPos.y - 20.0f;
                mDepthBias = 0.008f;
            }
            else if (name == SceneName::SimpleScene)
            {
                mLightUp = float3(0.0f, 1.0f, 0.0f);
                mLightPos = float3(0.0f, 5.0f, -10.0f);
                mRotation = float3(40.0f, 0.0f, 0.0f);
                mLightSize = float2(0.75f);
                mFovY = 55.0;
                mLightNearPlane = 8.0f;
                mDepthBias = 0.008f;
            }
            else if (name == SceneName::BistroExterior)
            {
                mLightUp = float3(0.0f, 1.0f, 0.0f);
                mLightPos = float3(-10.0f, 70.0f, -50.0f);
                mRotation = float3(40.0f, 0.0f, 0.0f);
                mLightSize = float2(15.0f);
                mFovY = 90.0;
                mLightNearPlane = 40.0f;
                mDepthBias = 0.01f;
            }
            else if (name == SceneName::BistroInterior)
            {
                mLightUp = float3(0, 1, 0);
                mLightPos = float3(4.955003, 2.819287, 6.114000);
                mRotation = float3(160.0f, -10.0f, 0.0f);
                mLightSize = float2(2.0f);
                mFovY = 125.0;
                mLightNearPlane = 1.5f;
                mDepthBias = 0.0005f;
            }
            else if (name == SceneName::OnePlaneScene || name == SceneName::ThreePlaneScene)
            {
                mLightUp = float3(0.0f, 0.0f, -1.0f);
                mLightPos = float3(0.0f, 10.0f, 0.0f);
                mRotation = float3(90.0f, 0.0f, 0.0f);
                mLightSize = float2(1.0f);
                mFovY = name == SceneName::OnePlaneScene ? 140.0f : 40.0f;
                mLightNearPlane = name == SceneName::OnePlaneScene ? 3.0f : 6.0f; 
                mDepthBias = 0.0005f;
            }
            else if (name == SceneName::Bicycle)
            {
                mLightUp = float3(0.0f, 1.0f, 0.0f);
                mLightPos = float3(7.0f, 8.0f, -12.0f);
                mRotation = float3(35.0f, -30.0f, 0.0f);
                mLightSize = float2(1.0f);
                mFovY = 35.0;
                mLightNearPlane = 12.0f;
                mDepthBias = 0.0009f;
            }
            else if (name == SceneName::PalmTrees)
            {
                mLightUp = float3(0.0f, 1.0f, 0.0f);
                mLightPos = float3(-2.0f, 26.0f, -5.0f);
                mRotation = float3(70.0f, 60.0f, 0.0f);
                mLightSize = float2(1.5f);
                mFovY = 81.0;
                mLightNearPlane = 6.0f;
                mDepthBias = 0.003f;
            }
        }

        // Only for Bistro Scenes
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
            pAreaLight->setScaling(float3(mLightSize / 2.0f, 1.0f));
            pAreaLight->setTransformMatrix(model);
        }

        void updateSceneEmissiveLight(const Scene::SharedPtr& pScene, RenderContext* pRenderContext)
        {
            // Get emissive light's mesh instance ID 
            const auto& pLightCollection = pScene->getLightCollection(pRenderContext);
            const auto& meshLights = pLightCollection->getMeshLights();
            uint instanceID = meshLights[0].instanceID;

            // Get all necessary data from Scene 
            const auto& pMeshInstance = pScene->getGeometryInstance(instanceID);
            const auto& meshBound = pScene->getMeshBounds(instanceID);
            const auto pAnimationController = pScene->getAnimationController();
            const auto& globalMatrices = pAnimationController->getGlobalMatrices();

            // Compute light position, light size
            float3 size = meshBound.extent(); // Bound box size of the mesh
            float3 lightCenter = meshBound.center();
            if (pScene->hasAnimation()) 
            {
                mObjectToWorld = globalMatrices[pMeshInstance.globalMatrixID];
                mLightSize = float2(size.x, size.y);
                mLightPos = mObjectToWorld * float4(lightCenter, 1.0f);
                //logInfo("size = " + to_string(size));
            }
            else
            {
                auto translation = glm::translate(glm::mat4(1.0f), mLightPos);
                auto rotationX = glm::rotate(glm::mat4(1.0f), glm::radians(mRotation.x), float3(1.0f, 0.0f, 0.0f));
                auto rotationY = glm::rotate(glm::mat4(1.0f), glm::radians(mRotation.y), float3(0.0f, 1.0f, 0.0f));
                auto rotationZ = glm::rotate(glm::mat4(1.0f), glm::radians(mRotation.z), float3(0.0f, 0.0f, 1.0f));
                mObjectToWorld = translation * rotationZ * rotationY * rotationX;

                float3 minPoint = glm::inverse(mObjectToWorld) * float4(meshBound.minPoint, 1.0f);
                float3 maxPoint = glm::inverse(mObjectToWorld) * float4(meshBound.maxPoint, 1.0f);
                //logInfo("minPoint = " + to_string(minPoint));
                //logInfo("maxPoint = " + to_string(maxPoint));
                size = maxPoint - minPoint; // Wrong calculation
                //logInfo("size = " + to_string(size));

                mLightSize = float2(size.x, size.y);
            }

            

            if (name == SceneName::OnePlaneScene)
            {
                const auto& pBlockerMeshInstance = pScene->getGeometryInstance(2);
                const auto& blockerMeshBound = pScene->getMeshBounds(2);
                float3 blockerCenter = meshBound.center();
                const auto& blockerTransform = globalMatrices[pBlockerMeshInstance.globalMatrixID];
                float3 blockerCenterW = blockerTransform * float4(blockerCenter, 1.0f);

                mLightNearPlane = mLightPos.y - blockerCenterW.y - 0.1f;
                //mLightNearPlane = 0.1f;
            }
        }
        
    };

}
