#pragma once

#include <CoreMinimal.h>

DECLARE_LOG_CATEGORY_EXTERN(LogNav3D, Log, All)

class FNav3DModule final : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
