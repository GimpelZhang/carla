// # Copyright 2023 Luma AI

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FLumaRuntimeModule : public IModuleInterface
{
public:
	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
