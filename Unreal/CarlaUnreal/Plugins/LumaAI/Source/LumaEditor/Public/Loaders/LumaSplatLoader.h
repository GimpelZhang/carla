// # Copyright 2023 Luma AI
#pragma once

#include "LumaSplatLoader.generated.h"

UCLASS()
class LUMAEDITOR_API ULumaSplatLoader : public UObject
{
	GENERATED_BODY()
public:
	static ULumaSplatLoader* Get();
	UObject* CreateFromFileInExistingPackage(FString FileName, UObject* ParentPackage);
};