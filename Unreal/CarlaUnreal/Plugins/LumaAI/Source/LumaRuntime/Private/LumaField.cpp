// # Copyright 2023 Luma AI

#include "LumaField.h"

#include "GenericPlatform/GenericPlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

#include <Engine/StaticMesh.h>
#include <Components/StaticMeshComponent.h>

#include "LumaEditor/Public/Loaders/RaymarchMaterialParameters.h"

ALumaField::ALumaField() : AActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
	
	SetActorEnableCollision(true);

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Default Scene Root"));
	RootComponent->SetWorldScale3D(FVector(1.0f));
	RootComponent->Mobility = EComponentMobility::Type::Movable;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> UnitSphere(
		TEXT("/LumaAIPlugin/Meshes/sphere"));

	StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Raymarch Static Mesh"));
	
	if (UnitSphere.Succeeded())
	{
		StaticMeshComponent->Mobility = EComponentMobility::Type::Movable;
		StaticMeshComponent->SetStaticMesh(UnitSphere.Object);
		StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
		StaticMeshComponent->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
		StaticMeshComponent->SetupAttachment(RootComponent);
	}

	ApplyVersionParams();
}

void ALumaField::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	ApplyVersionParams();
	
	if (HasAnyFlags(RF_ClassDefaultObject | RF_ArchetypeObject))
	{
		return;
	}

	if (IsInitialized)
	{
		return;
	}

	if (StaticMeshComponent && LumaBaseMaterial)
	{
		LumaMaterialInstance = UMaterialInstanceDynamic::Create((UMaterialInterface*) LumaBaseMaterial, (UObject*) this, FName("Material Instance"));
		ApplyShaderParams();
		StaticMeshComponent->SetMaterial(0, LumaMaterialInstance);
		IsInitialized = true;
	}

}

void ALumaField::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
	ApplyVersionParams();
}


void ALumaField::ApplyShaderParams() const
{
	if(!LumaMaterialInstance)
		return;
	
	LumaMaterialInstance->SetVectorParameterValue(LumaConstants::CropMin, CropMin);
	LumaMaterialInstance->SetVectorParameterValue(LumaConstants::CropMax, CropMax);
	LumaMaterialInstance->SetVectorParameterValue(LumaConstants::CropCenter, CropCenter);
	LumaMaterialInstance->SetVectorParameterValue(LumaConstants::CropRotation, CropRotation.Euler());	
	LumaMaterialInstance->SetVectorParameterValue(LumaConstants::Exposure, Exposure);
	LumaMaterialInstance->SetScalarParameterValue(LumaConstants::QualityMode, static_cast<float>(QualityMode));	
}

void ALumaField::ApplyVersionParams()
{
	if (Version < ELumaVersion::LatestVersion)
	{
		VersionInfo = "This blueprint was generated using an older version \n(";
		VersionInfo += *UEnum::GetValueAsString(Version);
		VersionInfo += ")\n of the Luma AI Plugin. To enable newer features supported by\n this version of this plugin, please re-import the Luma field!";
	}
	else
	{
		VersionInfo = "Blueprint version up-to-date with the current version of the Luma AI Plugin.";
	}

	if (Version >= ELumaVersion::V002_Added_Crop_And_Exposure_Controls)
	{
		HasCropSupport = true;
		HasExposureSupport = true;
	}

	if (Version >= ELumaVersion::V003_Crop_Rotation_And_Quality_Controls)
	{
		HasQualitySupport = true;
		HasCropRotationSupport = true;
	}
}

#if WITH_EDITOR

void ALumaField::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	const FName PropertyName = PropertyChangedEvent.GetPropertyName();
	const FName MemberPropertyName = PropertyChangedEvent.GetMemberPropertyName();
	ApplyVersionParams();

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaField, CropMin))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaField, CropMax))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaField, CropCenter))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaField, CropRotation))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaField, QualityMode))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaField, Exposure))
	{
		ApplyShaderParams();
		return;
	}
}
#endif	