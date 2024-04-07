// # Copyright 2023 Luma AI

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Math/IntVector.h"
#include "UObject/UnrealType.h"
#include "GameFramework/Actor.h"

#include "CoreMinimal.h"
#include "LocationVolume.h"
#include "Misc/Guid.h"

#include "LumaField.h"
#include "NiagaraComponent.h"
#include "LumaScene.generated.h"

USTRUCT()
struct FGaussianComponentChunk
{
	GENERATED_BODY()
	UTexture* col;
	UTexture* pos;
	UTexture* rot;
	UTexture* scale;
	TArray<FFloat16> col_data;
	TArray<FFloat16> pos_data;
	TArray<FFloat16> rot_data;
	TArray<FFloat16> scale_data;
	uint32 num;
	FVector3f center;
	FVector3f extent;
	uint32 texture_width;
	uint32 texture_height;
};

UCLASS(meta = (PrioritizeCategories ="Luma Transform"), HideCategories=("Hidden"))
class LUMARUNTIME_API ALumaScene : public AActor
{
	GENERATED_BODY()

public:

	constexpr static int32 NUM_NGC_COMPONENTS = 4 * 4 * 4;
	ALumaScene();

	// Empty constructor for subclass
	ALumaScene(bool dummy) {};

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	ALocationVolume* CropBox;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	ALocationVolume* CullBox1;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	ALocationVolume* CullBox2;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	ALocationVolume* CullBox3;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	ALocationVolume* CullBox4;

	virtual UNiagaraComponent* AddNGC(FString name, int idx)
	{
		//auto ngc = NewObject<UNiagaraComponent>(this, UNiagaraComponent::StaticClass(), FName(*name)); //CreateDefaultSubobject<UNiagaraComponent>(FName(*name));
		LumaNiagaraComponents[idx] = CreateDefaultSubobject<UNiagaraComponent>(FName(*name));
		LumaNiagaraComponents[idx]->SetupAttachment(RootComponent);
		return LumaNiagaraComponents[idx];
	}

	void CreateNGCComponents(int num_components);

	virtual void PostRegisterAllComponents() override;

	virtual bool ShouldTickIfViewportsOnly() const override;

	virtual void PostActorCreated() override;
	virtual void PostLoad() override;
	virtual void BeginPlay() override;
	virtual void PostInitProperties() override;
	virtual void PostInitializeComponents() override;
	virtual void PostReinitProperties() override;
	virtual void TickActor(float DeltaTime, ELevelTick TickType, FActorTickFunction& ThisTickFunction) override;

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Splat Scale")
	FVector3f SplatScale{1.0f, 1.0f, 1.0f};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Enable Crop Box")
	bool EnableCropBox{false};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Enable Cull Box 1")
	bool EnableCullBox1{false};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Enable Cull Box 2")
	bool EnableCullBox2{false};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Enable Cull Box 3")
	bool EnableCullBox3{false};

	UPROPERTY(EditAnywhere, Category="Luma", DisplayName="Enable Cull Box 4")
	bool EnableCullBox4{false};

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	UNiagaraSystem* LumaNiagaraSystem;

	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	TArray<UNiagaraComponent*> LumaNiagaraComponents;
	
	UPROPERTY(VisibleAnywhere, Category = "Hidden")
	TArray<FGaussianComponentChunk> LumaGaussianChunks;

	virtual void Serialize(FArchive& Ar) override;

	virtual void ApplyShaderParams() const;

	bool IsInitialized = false;
	int RefreshTicks = 0;

	UPROPERTY(VisibleAnywhere, Category = "Luma")
	ELumaVersion Version = ELumaVersion::V001_Initial_Release;

	UPROPERTY(VisibleAnywhere, Category = "Luma")
	FString VersionInfo;

	virtual void RefreshNiagaraComponents();

	virtual void CreateBox(ALocationVolume*& box, FString label, FVector scale);
	
	virtual void ApplyVersionParams();
	
	
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent);
#endif	
};



UCLASS(meta = (PrioritizeCategories ="Luma Transform"), HideCategories=("Hidden"))
class LUMARUNTIME_API ALumaSceneSingle : public ALumaScene
{
	GENERATED_BODY()

public:

	constexpr static int32 NUM_NGC_COMPONENTS = 1 * 1 * 1;
	ALumaSceneSingle();

};