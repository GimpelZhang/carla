// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/DepthCamera.h"
#include "Carla.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include "Carla/Sensor/PixelReader.h"

FActorDefinition ADepthCamera::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeCameraDefinition(TEXT("depth"));
}

ADepthCamera::ADepthCamera(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  AddPostProcessingMaterial(
      TEXT("Material'/Carla/PostProcessingMaterials/PhysicLensDistortion.PhysicLensDistortion'"));
  AddPostProcessingMaterial(
#if PLATFORM_LINUX
      TEXT("Material'/Carla/PostProcessingMaterials/DepthEffectMaterial_GLSL.DepthEffectMaterial_GLSL'")
#else
      TEXT("Material'/Carla/PostProcessingMaterials/DepthEffectMaterial.DepthEffectMaterial'")
#endif
  );
}

void ADepthCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ADepthCamera::PostPhysTick);
  Super::PostPhysTick(World, TickType, DeltaSeconds);

  auto FrameIndex = FCarlaEngine::GetFrameCounter();
  ImageUtil::ReadSensorImageDataAsyncFColor(*this, [this, FrameIndex](
    TArrayView<const FColor> Pixels,
    FIntPoint Size) -> bool
  {
    SendImageDataToClient(*this, Pixels, FrameIndex);
    return true;
  });
}
