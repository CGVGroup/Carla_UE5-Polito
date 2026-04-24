// Copyright (c) Meta Platforms, Inc. and affiliates.

#include "OculusXRTelemetry.h"
#include "OculusXRHMDModule.h"
#include "OculusXRTelemetryPrivacySettings.h"
#include "Async/Async.h"
#include "GeneralProjectSettings.h"

namespace OculusXRTelemetry
{
	namespace
	{
		const char* TelemetrySource = "UE5Integration";
	}

	bool IsActive()
	{
#if OCULUS_HMD_SUPPORTED_PLATFORMS
		if constexpr (FTelemetryBackend::IsNullBackend())
		{
			return false;
		}
		if (FOculusXRHMDModule::Get().IsOVRPluginAvailable() && FOculusXRHMDModule::GetPluginWrapper().IsInitialized())
		{
			return true;
		}
#endif // OCULUS_HMD_SUPPORTED_PLATFORMS
		return false;
	}

	void IfActiveThen(TUniqueFunction<void()> Function)
	{
		AsyncTask(ENamedThreads::GameThread, [F = MoveTemp(Function)]() {
			if (IsActive())
			{
				F();
			}
		});
	}

	void PropagateTelemetryConsent()
	{
#ifdef WITH_EDITOR
		// The consent mechanism only deals with in-editor preferences by developer. In runtime, this consent doesn't matter.
		bool HasConsent = false;
		if (const auto EditorPrivacySettings = GetDefault<UOculusXRTelemetryPrivacySettings>())
		{
			HasConsent = EditorPrivacySettings->bIsEnabled;
		}
		if (FOculusXRHMDModule::Get().IsOVRPluginAvailable() && FOculusXRHMDModule::GetPluginWrapper().IsInitialized())
		{
			FOculusXRHMDModule::GetPluginWrapper().QplSetConsent(HasConsent);
			FOculusXRHMDModule::GetPluginWrapper().SetDeveloperTelemetryConsent(HasConsent);
		}
#endif
	}

	FString GetProjectId()
	{
		const UGeneralProjectSettings& ProjectSettings = *GetDefault<UGeneralProjectSettings>();
		return ProjectSettings.ProjectID.ToString();
	}

	bool IsConsentGiven()
	{
#ifdef WITH_EDITOR
		if (const UOculusXRTelemetryPrivacySettings* EditorPrivacySettings = GetDefault<UOculusXRTelemetryPrivacySettings>())
		{
			return EditorPrivacySettings->bIsEnabled;
		}
#endif
		return false;
	}

	void SendEvent(const TCHAR* EventName, float Param)
	{
		const FString StrVal = FString::Printf(TEXT("%f"), Param);
		SendEvent(EventName, *StrVal);
	}

	void SendEvent(const TCHAR* EventName, bool bParam)
	{
		SendEvent(EventName, bParam ? TEXT("true") : TEXT("false"));
	}

	void SendEvent(const TCHAR* EventName, const TCHAR* Param)
	{
		if (IsActive())
		{
			FOculusXRHMDModule::GetPluginWrapper().SendEvent2(TCHAR_TO_ANSI(EventName), TCHAR_TO_ANSI(Param), TelemetrySource);
		}
	}

} // namespace OculusXRTelemetry
