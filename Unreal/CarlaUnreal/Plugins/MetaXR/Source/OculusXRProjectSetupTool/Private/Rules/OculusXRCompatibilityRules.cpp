// Copyright (c) Meta Platforms, Inc. and affiliates.

#include "OculusXRCompatibilityRules.h"
#include "CoreMinimal.h"
#include "AndroidRuntimeSettings.h"
#include "AndroidSDKSettings.h"
#include "GeneralProjectSettings.h"
#include "OculusXRRuleProcessorSubsystem.h"
#include "GameFramework/InputSettings.h"
#include "OculusXRHMDRuntimeSettings.h"
#include "OculusXRPSTUtils.h"

#define LOCTEXT_NAMESPACE "OculusXRCompatibilityRules"
namespace
{
	constexpr int32 MinimumAndroidAPILevel = 32;
	constexpr int32 TargetAndroidAPILevel = 32;
	constexpr char AndroidNDKVersionNumber[] = "25.1.8937393";
	constexpr char AndroidSDKAPILevel[] = "android-29";
	constexpr int32 AndroidSDKAPILevelInt = 29;
	constexpr char AndroidNDKAPILevel[] = "android-29";
	constexpr int32 AndroidNDKAPILevelInt = 29;
} // namespace

namespace OculusXRCompatibilityRules
{

	FUseAndroidSDKMinimumRule::FUseAndroidSDKMinimumRule()
		: ISetupRule(
			  "Compatibility_UseAndroidSDKMinimum",
			  LOCTEXT("UseAndroidSDKMinimum_DisplayName", "Use Android SDK Minimum Version"),
			  FText::Format(
				  LOCTEXT("UseAndroidSDKMinimum_Description", "Minimum Android API level must be at least {0}."),
				  MinimumAndroidAPILevel),
			  ESetupRuleCategory::Compatibility,
			  ESetupRuleSeverity::Critical,
			  MetaQuest_All) {}

	bool FUseAndroidSDKMinimumRule::IsApplied() const
	{
		const UAndroidRuntimeSettings* Settings = GetMutableDefault<UAndroidRuntimeSettings>();

		return Settings->MinSDKVersion >= MinimumAndroidAPILevel;
	}

	void FUseAndroidSDKMinimumRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, MinSDKVersion, MinimumAndroidAPILevel);
		OutShouldRestartEditor = false;
	}

	FUseAndroidSDKTargetRule::FUseAndroidSDKTargetRule()
		: ISetupRule(
			  "Compatibility_UseAndroidSDKTarget",
			  LOCTEXT("UseAndroidSDKTarget_DisplayName", "Use Android SDK Target Version"),
			  FText::Format(
				  LOCTEXT("UseAndroidSDKTarget_Description", "Target Android API level must be at least {0}."),
				  TargetAndroidAPILevel),
			  ESetupRuleCategory::Compatibility,
			  ESetupRuleSeverity::Critical,
			  MetaQuest_All) {}

	bool FUseAndroidSDKTargetRule::IsApplied() const
	{
		const UAndroidRuntimeSettings* Settings = GetMutableDefault<UAndroidRuntimeSettings>();

		return Settings->TargetSDKVersion >= TargetAndroidAPILevel;
	}

	void FUseAndroidSDKTargetRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, TargetSDKVersion, TargetAndroidAPILevel);
		OutShouldRestartEditor = false;
	}
	FUseAndroidSDKLevelRule::FUseAndroidSDKLevelRule()
		: ISetupRule(
			  "Compatibility_UseAndroidSDKLevel",
			  LOCTEXT("UseAndroidSDKLevel_DisplayName", "Use Android SDK Level"),
			  FText::Format(
				  LOCTEXT("UseAndroidSDKLevel_Description", "Android SDK level should be set to {0} or higher prior to packaging apks."),
				  FText::AsCultureInvariant(AndroidSDKAPILevel)),
			  TEXT("https://developer.oculus.com/blog/meta-quest-apps-android-12l-june-30/"),
			  ESetupRuleCategory::Compatibility,
			  ESetupRuleSeverity::Critical,
			  MetaQuest_All,
			  true) {}

	bool FUseAndroidSDKLevelRule::IsApplied() const
	{
		const UAndroidSDKSettings* Settings = GetMutableDefault<UAndroidSDKSettings>();
		FString SDKAPILevel = Settings->SDKAPILevel;

		if (SDKAPILevel.IsEmpty())
		{
			return false;
		}
		if (SDKAPILevel.Equals(TEXT("latest")) || SDKAPILevel.Equals(TEXT("matchndk")))
		{
			return true;
		}
		if (!SDKAPILevel.Left(8).Equals(TEXT("android-")))
		{
			return false;
		}
		if (FCString::Atoi(*SDKAPILevel.Right(2)) < AndroidSDKAPILevelInt)
		{
			return false;
		}

		return true;
	}

	void FUseAndroidSDKLevelRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OutShouldRestartEditor = false;
		UAndroidSDKSettings* Settings = GetMutableDefault<UAndroidSDKSettings>();
		Settings->SDKAPILevel = FText::AsCultureInvariant(AndroidSDKAPILevel).ToString();
	}

	FUseAndroidNDKLevelRule::FUseAndroidNDKLevelRule()
		: ISetupRule(
			  "Compatibility_UseAndroidNDKLevel",
			  LOCTEXT("UseAndroidNDKLevel_DisplayName", "Use Android NDK Level"),
			  FText::Format(
				  LOCTEXT("UseAndroidNDKLevel_Description", "Android NDK level should be set to {0} or higher prior to packaging apks."),
				  FText::AsCultureInvariant(AndroidNDKAPILevel)),
			  TEXT("https://developer.oculus.com/blog/meta-quest-apps-must-target-android-10-starting-september-29/"),
			  ESetupRuleCategory::Compatibility,
			  ESetupRuleSeverity::Critical,
			  MetaQuest_All,
			  true) {}

	bool FUseAndroidNDKLevelRule::IsApplied() const
	{
		const UAndroidSDKSettings* Settings = GetMutableDefault<UAndroidSDKSettings>();
		FString NDKAPILevel = Settings->NDKAPILevel;

		if (NDKAPILevel.IsEmpty())
		{
			return false;
		}
		if (NDKAPILevel.Equals(TEXT("latest")))
		{
			return true;
		}
		if (!NDKAPILevel.Left(8).Equals(TEXT("android-")))
		{
			return false;
		}
		if (FCString::Atoi(*NDKAPILevel.Right(2)) < AndroidNDKAPILevelInt)
		{
			return false;
		}

		return true;
	}

	void FUseAndroidNDKLevelRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OutShouldRestartEditor = false;
		UAndroidSDKSettings* Settings = GetMutableDefault<UAndroidSDKSettings>();
		Settings->NDKAPILevel = FText::AsCultureInvariant(AndroidNDKAPILevel).ToString();
	}

	bool FUseArm64CPURule::IsApplied() const
	{
		const UAndroidRuntimeSettings* Settings = GetMutableDefault<UAndroidRuntimeSettings>();

		return Settings->bBuildForArm64 && !Settings->bBuildForX8664;
	}

	void FUseArm64CPURule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, bBuildForArm64, true);
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, bBuildForX8664, false);
		OutShouldRestartEditor = false;
	}
	bool FEnablePackageForMetaQuestRule::IsApplied() const
	{
		const UAndroidRuntimeSettings* Settings = GetMutableDefault<UAndroidRuntimeSettings>();

		return Settings->bPackageForMetaQuest && !Settings->bSupportsVulkanSM5 && !Settings->bBuildForES31 && Settings->ExtraApplicationSettings.Find("com.oculus.supportedDevices") != INDEX_NONE;
	}

	void FEnablePackageForMetaQuestRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, bPackageForMetaQuest, true);
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, bSupportsVulkanSM5, false);
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, bBuildForES31, false);

		UAndroidRuntimeSettings* Settings = GetMutableDefault<UAndroidRuntimeSettings>();
		if (Settings->ExtraApplicationSettings.Find("com.oculus.supportedDevices") == INDEX_NONE)
		{
			const FString SupportedDevicesValue("quest|quest2|questpro");
			Settings->ExtraApplicationSettings.Append("<meta-data android:name=\"com.oculus.supportedDevices\" android:value=\"" + SupportedDevicesValue + "\" />");
			Settings->UpdateSinglePropertyInConfigFile(Settings->GetClass()->FindPropertyByName(GET_MEMBER_NAME_CHECKED(UAndroidRuntimeSettings, ExtraApplicationSettings)), Settings->GetDefaultConfigFilename());
		}

		OutShouldRestartEditor = false;
	}

	bool FQuest2SupportedDeviceRule::IsApplied() const
	{
		const UOculusXRHMDRuntimeSettings* Settings = GetMutableDefault<UOculusXRHMDRuntimeSettings>();

		return Settings->SupportedDevices.Contains(EOculusXRSupportedDevices::Quest2);
	}

	void FQuest2SupportedDeviceRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		UOculusXRHMDRuntimeSettings* Settings = GetMutableDefault<UOculusXRHMDRuntimeSettings>();

		Settings->SupportedDevices.Add(EOculusXRSupportedDevices::Quest2);
		// UpdateSinglePropertyInConfigFile does not support arrays
		Settings->SaveConfig(CPF_Config, *Settings->GetDefaultConfigFilename());
		OutShouldRestartEditor = false;
	}

	bool FQuestProSupportedDeviceRule::IsApplied() const
	{
		const UOculusXRHMDRuntimeSettings* Settings = GetMutableDefault<UOculusXRHMDRuntimeSettings>();

		return Settings->SupportedDevices.Contains(EOculusXRSupportedDevices::QuestPro);
	}

	void FQuestProSupportedDeviceRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		UOculusXRHMDRuntimeSettings* Settings = GetMutableDefault<UOculusXRHMDRuntimeSettings>();

		Settings->SupportedDevices.Add(EOculusXRSupportedDevices::QuestPro);
		// UpdateSinglePropertyInConfigFile does not support arrays
		Settings->SaveConfig(CPF_Config, *Settings->GetDefaultConfigFilename());
		OutShouldRestartEditor = false;
	}

	bool FQuest3SupportedDeviceRule::IsApplied() const
	{
		const UOculusXRHMDRuntimeSettings* Settings = GetMutableDefault<UOculusXRHMDRuntimeSettings>();

		return Settings->SupportedDevices.Contains(EOculusXRSupportedDevices::Quest3);
	}

	void FQuest3SupportedDeviceRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		UOculusXRHMDRuntimeSettings* Settings = GetMutableDefault<UOculusXRHMDRuntimeSettings>();

		Settings->SupportedDevices.Add(EOculusXRSupportedDevices::Quest3);
		// UpdateSinglePropertyInConfigFile does not support arrays
		Settings->SaveConfig(CPF_Config, *Settings->GetDefaultConfigFilename());
		OutShouldRestartEditor = false;
	}

	bool FEnableFullscreenRule::IsApplied() const
	{
		const UAndroidRuntimeSettings* Settings = GetMutableDefault<UAndroidRuntimeSettings>();

		return Settings->bFullScreen;
	}

	void FEnableFullscreenRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OCULUSXR_UPDATE_SETTINGS(UAndroidRuntimeSettings, bFullScreen, true);
		OutShouldRestartEditor = false;
	}

	bool FEnableStartInVRRule::IsApplied() const
	{
		const UGeneralProjectSettings* Settings = GetDefault<UGeneralProjectSettings>();

		return Settings->bStartInVR != 0;
	}

	void FEnableStartInVRRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OCULUSXR_UPDATE_SETTINGS(UGeneralProjectSettings, bStartInVR, true);
		OutShouldRestartEditor = false;
	}

	bool FDisableTouchInterfaceRule::IsApplied() const
	{
		const UInputSettings* Settings = GetDefault<UInputSettings>();

		return Settings->DefaultTouchInterface.IsNull();
	}

	void FDisableTouchInterfaceRule::ApplyImpl(bool& OutShouldRestartEditor)
	{
		OCULUSXR_UPDATE_SETTINGS(UInputSettings, DefaultTouchInterface, nullptr);
		OutShouldRestartEditor = false;
	}
} // namespace OculusXRCompatibilityRules

#undef LOCTEXT_NAMESPACE
