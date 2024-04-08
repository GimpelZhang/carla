// # Copyright 2023 Luma AI

#include "LumaScene.h"

#include "LocationVolume.h"
#include "GenericPlatform/GenericPlatformTime.h"
#include "Materials/MaterialInterface.h"
#include "LumaEditor/Public/Loaders/RaymarchMaterialParameters.h"
#include "Engine/World.h"
#include "NiagaraSystem.h"

#if WITH_EDITOR
#include "ActorFactories/ActorFactory.h"
#include "Builders/CubeBuilder.h"
#endif



// Sets default values
ALumaScene::ALumaScene()
{
	
	if (LumaNiagaraComponents.IsEmpty())
	{
		PrimaryActorTick.bCanEverTick = true;
		PrimaryActorTick.bStartWithTickEnabled = true;

		RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Default Scene Root"));
		RootComponent->SetWorldScale3D(FVector(1.0f));
		RootComponent->Mobility = EComponentMobility::Type::Movable;

		CreateNGCComponents(ALumaScene::NUM_NGC_COMPONENTS);
		SetActorTickInterval(0.1);
	}
	
}


void ALumaScene::CreateNGCComponents(int num_components)
{
	FString ngc_name_base = "LumaNGC";

	LumaNiagaraComponents.SetNum(num_components);
	for (int i = 0; i < num_components; i++)
	{
		AddNGC(ngc_name_base + FString::FromInt(i), i);
	}
}

// Called after registering all components. This is the last action performed before editor window is spawned and before BeginPlay.
void ALumaScene::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	ApplyVersionParams();
}

bool ALumaScene::ShouldTickIfViewportsOnly() const
{
	return true;
}

void ALumaScene::PostActorCreated()
{
	Super::PostActorCreated();
	RefreshTicks = 5;
}

void ALumaScene::PostLoad()
{
	Super::PostLoad();
	RefreshTicks = 10;
}

void ALumaScene::BeginPlay()
{
	Super::BeginPlay();
	RefreshTicks = 1;
}

void ALumaScene::PostInitProperties()
{
	Super::PostInitProperties();
}

void ALumaScene::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	RefreshTicks = 5;
}

void ALumaScene::PostReinitProperties()
{
	Super::PostReinitProperties();
}

void ALumaScene::TickActor(float DeltaTime, ELevelTick TickType, FActorTickFunction& ThisTickFunction)
{
	Super::TickActor(DeltaTime, TickType, ThisTickFunction);
	if(RefreshTicks > 0)
	{
		RefreshTicks--;
		if(RefreshTicks == 0)
			RefreshNiagaraComponents();
	}
}

void ALumaScene::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
	ApplyVersionParams();
}


void ALumaScene::ApplyShaderParams() const
{
	for(auto& ngc : LumaNiagaraComponents)
	{
		if(!ngc)
			continue;

		ngc->SetVariableVec3(LumaConstants::GSSplatScale, {SplatScale.X, SplatScale.Y, SplatScale.Z});

		if(CropBox)
		{
			ngc->SetVariableObject(LumaConstants::GSCropActor, CropBox);
			ngc->SetVariableBool(LumaConstants::GSCropActorEnabled, EnableCropBox);
		}

		if(CullBox1)
		{
			ngc->SetVariableObject(LumaConstants::GSCullActor1, CullBox1);
			ngc->SetVariableBool(LumaConstants::GSCullActorEnabled1, EnableCullBox1);
		}

		if(CullBox2)
		{
			ngc->SetVariableObject(LumaConstants::GSCullActor2, CullBox2);
			ngc->SetVariableBool(LumaConstants::GSCullActorEnabled2, EnableCullBox2);
		}

		if(CullBox3)
		{
			ngc->SetVariableObject(LumaConstants::GSCullActor3, CullBox3);
			ngc->SetVariableBool(LumaConstants::GSCullActorEnabled3, EnableCullBox3);
		}

		if(CullBox4)
		{
			ngc->SetVariableObject(LumaConstants::GSCullActor4, CullBox4);
			ngc->SetVariableBool(LumaConstants::GSCullActorEnabled4, EnableCullBox4);
		}
		
	}
}

void ALumaScene::RefreshNiagaraComponents()
{

	CreateBox(CropBox, "CropBox1_", {64, 64, 64});
	CreateBox(CullBox1, "CullBox1_", {1, 1, 1});	
	CreateBox(CullBox2, "CullBox2_", {1, 1, 1});	
	CreateBox(CullBox3, "CullBox3_", {1, 1, 1});	
	CreateBox(CullBox4, "CullBox4_", {1, 1, 1});	
	
	ApplyShaderParams();
	
#if WITH_EDITOR

	if(LumaNiagaraComponents.IsEmpty())
		return;
	
	for (int i = 0; i < LumaNiagaraComponents.Num(); i++)
	{
		if(LumaNiagaraComponents[i]->GetAsset() == nullptr)
			continue;
		
		LumaNiagaraComponents[i]->GetAsset()->EnsureFullyLoaded();
		LumaNiagaraComponents[i]->GetAsset()->InvalidateCachedData();
		LumaNiagaraComponents[i]->GetAsset()->RequestCompile(false);
	}
#endif
}

void ALumaScene::CreateBox(ALocationVolume*& box, FString label, FVector scale)
{
	if(box)
		return;

	UWorld* World = GetWorld();

	box = World->SpawnActor<ALocationVolume>(GetActorLocation(), GetActorRotation());
	
	if (box)
	{
		box->bIsRuntime = true;
		box->SetActorEnableCollision(false);
		SetActorEnableCollision(false);
		box->GetRootComponent()->SetMobility(EComponentMobility::Movable);
		box->GetRootComponent()->SetWorldScale3D(scale);

#if WITH_EDITOR
		box->SetActorLabel(label + GetActorLabel());
		UCubeBuilder* Builder = NewObject<UCubeBuilder>();
		Builder->X = 200.0f;
		Builder->Y = 200.0f;
		Builder->Z = 200.0f;
		UActorFactory::CreateBrushForVolumeActor((AVolume*)box, Builder);
#endif
			
		box->AttachToActor(this, FAttachmentTransformRules::KeepWorldTransform);
	}
	
}

void ALumaScene::ApplyVersionParams()
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

}

#if WITH_EDITOR


void ALumaScene::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	const FName MemberPropertyName = PropertyChangedEvent.GetMemberPropertyName();
	ApplyVersionParams();

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaScene, EnableCropBox))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaScene, SplatScale))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaScene, EnableCullBox1))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaScene, EnableCullBox2))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaScene, EnableCullBox3))
	{
		ApplyShaderParams();
		return;
	}

	if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ALumaScene, EnableCullBox4))
	{
		ApplyShaderParams();
		return;
	}
}

#endif	

ALumaSceneSingle::ALumaSceneSingle() : ALumaScene(false)
{
	if (LumaNiagaraComponents.IsEmpty())
	{
		PrimaryActorTick.bCanEverTick = true;
		PrimaryActorTick.bStartWithTickEnabled = true;

		RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Default Scene Root"));
		RootComponent->SetWorldScale3D(FVector(1.0f));
		RootComponent->Mobility = EComponentMobility::Type::Movable;

		CreateNGCComponents(ALumaSceneSingle::NUM_NGC_COMPONENTS);
		SetActorTickInterval(0.2);
	}
}
