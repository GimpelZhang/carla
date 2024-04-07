// # Copyright 2023 Luma AI

#pragma once

#include "Modules/ModuleManager.h"

class FLumaEditorModule : public IModuleInterface
{
public:

	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
