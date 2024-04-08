// # Copyright 2023 Luma AI
#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Math/IntVector.h"
#include "UObject/UnrealType.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInstanceDynamic.h"

#include "CoreMinimal.h"
#include "Misc/Guid.h"

#include "LumaField.generated.h"

UENUM(BlueprintType)
enum class ELumaVersion : uint8
{
	// Before any version changes were made in the plugin
	V001_Initial_Release = 0,
	V002_Added_Crop_And_Exposure_Controls = 1,
	V003_Crop_Rotation_And_Quality_Controls = 2,

	// -----<new versions can be added above this line>-------------------------------------------------
	// Add a new version if anything important changes in the Blueprint that might require users to update.
	VersionPlusOne,
	LatestVersion = VersionPlusOne - 1
};

UENUM(BlueprintType)
enum class ELumaQualityMode : uint8
{
	Minimal = 0,
	VeryLow,
	Low,
	Mid,
	High,
	VeryHigh,
	Extreme,
};

// Raymarching parameters
struct QualityData
{
	float min_step_size;
	float lindisp_step;
	int max_steps;
	float stop_thresh;
	float expon_step;
};

constexpr QualityData get_luma_quality_settings(ELumaQualityMode mode)
{
	switch (mode)
	{
		case ELumaQualityMode::Minimal: return QualityData{0.05, 0.03, 12, 0.4, 0.04};
		case ELumaQualityMode::VeryLow: return QualityData{0.02, 0.01, 24, 0.3, 0.01};
		case ELumaQualityMode::Low: return QualityData{0.01, 0.005, 48, 0.25, 0.008};
		case ELumaQualityMode::Mid: return QualityData{0.005, 0.0012, 64, 0.2, 0.004};
		case ELumaQualityMode::High: return QualityData{0.0025, 0.0006, 96, 0.15, 0.002};
		case ELumaQualityMode::VeryHigh: return QualityData{0.0005, 0.00012, 128, 0.1, 0.001};
		case ELumaQualityMode::Extreme: return QualityData{0.0001, 0.00006, 256, 0.05, 0.001};
		default: return QualityData{0.005, 0.0012, 64};
	}
}

UCLASS(meta = (PrioritizeCategories ="Luma Transform"), HideCategories=("Hidden"))
class LUMARUNTIME_API ALumaField : public AActor
{
	GENERATED_BODY()

public:
	ALumaField();

	virtual void PostRegisterAllComponents() override;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	UStaticMeshComponent* StaticMeshComponent;
	
	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	UMaterialInstanceDynamic* LumaMaterialInstance = nullptr;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	UMaterial* LumaBaseMaterial = nullptr;

	/** Crop bounding box minimum. -2.0 extends towards the full range of the NeRF, -1.0 extends as far as the Blueprint size. */
	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Crop Bounding Box Minimum",
		meta=(EditCondition="HasCropSupport", EditConditionHides, ClampMin = "-2.0", ClampMax = "2.0", UIMin = "-2.0", UIMax = "2.0"))
	FVector3f CropMin{-2, -2, -2};

	/** Crop bounding box maximum. 2.0 extends towards the full range of the NeRF, 1.0 extends as far as the Blueprint size. */
	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Crop Bounding Box Maximum",
		meta=(EditCondition="HasCropSupport", EditConditionHides, ClampMin = "-2.0", ClampMax = "2.0", UIMin = "-2.0", UIMax = "2.0"))
	FVector3f CropMax{2, 2, 2};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Crop Bounding Center",
		meta=(EditCondition="HasCropRotationSupport", EditConditionHides, ClampMin = "-1.0", ClampMax = "1.0", UIMin = "-1.0", UIMax = "1.0"))
	FVector3f CropCenter{0, 0, 0};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Crop Rotation",
		meta=(EditCondition="HasCropRotationSupport", EditConditionHides))
	FRotator3d CropRotation;

	/** Exposure controls for this NeRF instance. Default is 1.0, 1.0, 1.0. */
	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Exposure", meta=(EditCondition="HasExposureSupport", EditConditionHides))
	FLinearColor Exposure{1, 1, 1, 1};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Quality", meta=(EditCondition="HasQualitySupport", EditConditionHides))
	ELumaQualityMode QualityMode = ELumaQualityMode::Mid;

	void Serialize(FArchive& Ar) override;

	void ApplyShaderParams() const;

	bool IsInitialized = false;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	bool LoaderFlagsSet = false;
	
	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	bool HasCropSupport = false;
	
	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	bool HasExposureSupport = false;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	bool HasQualitySupport = false;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	bool HasCropRotationSupport = false;

	UPROPERTY(VisibleAnywhere, Category = "Luma")
	ELumaVersion Version = ELumaVersion::V001_Initial_Release;

	UPROPERTY(VisibleAnywhere, Category = "Luma")
	FString VersionInfo;
	
	
	void ApplyVersionParams();
	
#if WITH_EDITOR
	void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent);
#endif	
};
