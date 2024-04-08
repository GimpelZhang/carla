// # Copyright 2023 Luma AI
#pragma once

#include "LumaLoader.generated.h"

UCLASS()
class LUMAEDITOR_API ULumaLoader : public UObject
{
	GENERATED_BODY()
public:
	static ULumaLoader* Get();
	UObject* CreateVolumeFromFileInExistingPackage(FString FileName, UObject* ParentPackage);
};