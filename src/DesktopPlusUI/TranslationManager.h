#pragma once

#include <string>
#include <vector>

#include "imgui.h"

enum TRMGRStrID
{
    tstr_SettingsWindowTitle,
    tstr_SettingsCatInterface,
    tstr_SettingsCatEnvironment,
    tstr_SettingsCatProfiles,
    tstr_SettingsCatActions,
    tstr_SettingsCatKeyboard,
    tstr_SettingsCatMouse,
    tstr_SettingsCatLaserPointer,
    tstr_SettingsCatWindowOverlays,
    tstr_SettingsCatBrowser,
    tstr_SettingsCatPerformance,
    tstr_SettingsCatVersionInfo,
    tstr_SettingsCatWarnings,
    tstr_SettingsCatStartup,
    tstr_SettingsCatTroubleshooting,
    tstr_SettingsWarningPrefix,
    tstr_SettingsWarningCompositorResolution,
    tstr_SettingsWarningCompositorQuality,
    tstr_SettingsWarningProcessElevated,
    tstr_SettingsWarningElevatedMode,
    tstr_SettingsWarningElevatedProcessFocus,
    tstr_SettingsWarningBrowserMissing,
    tstr_SettingsWarningBrowserMismatch,
    tstr_SettingsWarningUIAccessLost,
    tstr_SettingsWarningOverlayCreationErrorLimit,
    tstr_SettingsWarningOverlayCreationErrorOther,              //%ERRORNAME% == VROverlay::GetOverlayErrorNameFromEnum()
    tstr_SettingsWarningGraphicsCaptureError,                   //%ERRORCODE% == WinRT HRESULT in hex notation
    tstr_SettingsWarningAppProfileActive,                       //%APPNAME% == name of active application
    tstr_SettingsWarningConfigMigrated,
    tstr_SettingsWarningMenuDontShowAgain,
    tstr_SettingsWarningMenuDismiss,
    tstr_SettingsInterfaceLanguage,
    tstr_SettingsInterfaceLanguageCommunity,                    //%AUTHOR% == Language Author string
    tstr_SettingsInterfaceLanguageIncompleteWarning,
    tstr_SettingsInterfaceAdvancedSettings,
    tstr_SettingsInterfaceAdvancedSettingsTip,
    tstr_SettingsInterfaceBlankSpaceDrag,
    tstr_SettingsInterfacePersistentUI,
    tstr_SettingsInterfacePersistentUIManage,
    tstr_SettingsInterfaceDesktopButtons,
    tstr_SettingsInterfaceDesktopButtonsNone,
    tstr_SettingsInterfaceDesktopButtonsIndividual,
    tstr_SettingsInterfaceDesktopButtonsCycle,
    tstr_SettingsInterfaceDesktopButtonsAddCombined,
    tstr_SettingsInterfacePersistentUIHelp,
    tstr_SettingsInterfacePersistentUIHelp2,
    tstr_SettingsInterfacePersistentUIWindowsHeader,
    tstr_SettingsInterfacePersistentUIWindowsSettings,
    tstr_SettingsInterfacePersistentUIWindowsProperties,
    tstr_SettingsInterfacePersistentUIWindowsKeyboard,
    tstr_SettingsInterfacePersistentUIWindowsStateGlobal,
    tstr_SettingsInterfacePersistentUIWindowsStateDashboardTab,
    tstr_SettingsInterfacePersistentUIWindowsStateVisible,
    tstr_SettingsInterfacePersistentUIWindowsStatePinned,
    tstr_SettingsInterfacePersistentUIWindowsStatePosition,
    tstr_SettingsInterfacePersistentUIWindowsStatePositionReset,
    tstr_SettingsInterfacePersistentUIWindowsStateSize,
    tstr_SettingsInterfacePersistentUIWindowsStateLaunchRestore,
    tstr_SettingsEnvironmentBackgroundColor,
    tstr_SettingsEnvironmentBackgroundColorDispModeNever,
    tstr_SettingsEnvironmentBackgroundColorDispModeDPlusTab,
    tstr_SettingsEnvironmentBackgroundColorDispModeAlways,
    tstr_SettingsEnvironmentDimInterface,
    tstr_SettingsEnvironmentDimInterfaceTip,
    tstr_SettingsProfilesOverlays,
    tstr_SettingsProfilesApps,
    tstr_SettingsProfilesManage,
    tstr_SettingsProfilesOverlaysHeader,
    tstr_SettingsProfilesOverlaysNameDefault,
    tstr_SettingsProfilesOverlaysNameNew,
    tstr_SettingsProfilesOverlaysNameNewBase,                   //%ID% == ID of profile, increased if previous number is already taken
    tstr_SettingsProfilesOverlaysProfileLoad,
    tstr_SettingsProfilesOverlaysProfileAdd,
    tstr_SettingsProfilesOverlaysProfileSave,
    tstr_SettingsProfilesOverlaysProfileDelete,
    tstr_SettingsProfilesOverlaysProfileDeleteConfirm,
    tstr_SettingsProfilesOverlaysProfileFailedLoad,
    tstr_SettingsProfilesOverlaysProfileFailedDelete,
    tstr_SettingsProfilesOverlaysProfileAddSelectHeader,
    tstr_SettingsProfilesOverlaysProfileAddSelectEmpty,
    tstr_SettingsProfilesOverlaysProfileAddSelectDo,
    tstr_SettingsProfilesOverlaysProfileAddSelectAll,
    tstr_SettingsProfilesOverlaysProfileAddSelectNone,
    tstr_SettingsProfilesOverlaysProfileSaveSelectHeader,
    tstr_SettingsProfilesOverlaysProfileSaveSelectName,
    tstr_SettingsProfilesOverlaysProfileSaveSelectNameErrorBlank,
    tstr_SettingsProfilesOverlaysProfileSaveSelectNameErrorTaken,
    tstr_SettingsProfilesOverlaysProfileSaveSelectHeaderList, 
    tstr_SettingsProfilesOverlaysProfileSaveSelectDo,
    tstr_SettingsProfilesOverlaysProfileSaveSelectDoFailed,
    tstr_SettingsProfilesAppsHeader,
    tstr_SettingsProfilesAppsHeaderNoVRTip,
    tstr_SettingsProfilesAppsListEmpty,
    tstr_SettingsProfilesAppsProfileHeaderActive,
    tstr_SettingsProfilesAppsProfileEnabled,
    tstr_SettingsProfilesAppsProfileOverlayProfile,
    tstr_SettingsProfilesAppsProfileActionEnter,
    tstr_SettingsProfilesAppsProfileActionLeave,
    tstr_SettingsActionsManage,
    tstr_SettingsActionsManageButton,
    tstr_SettingsActionsButtonsOrderDefault,
    tstr_SettingsActionsButtonsOrderOverlayBar,
    tstr_SettingsActionsShowBindings,
    tstr_SettingsActionsActiveShortcuts,
    tstr_SettingsActionsActiveShortcutsTip,
    tstr_SettingsActionsActiveShortuctsHome,
    tstr_SettingsActionsActiveShortuctsBack,
    tstr_SettingsActionsGlobalShortcuts,
    tstr_SettingsActionsGlobalShortcutsTip,
    tstr_SettingsActionsGlobalShortcutsEntry,                   //%ID% == ID of shortcut, starts with 1
    tstr_SettingsActionsGlobalShortcutsAdd,
    tstr_SettingsActionsGlobalShortcutsRemove,
    tstr_SettingsActionsHotkeys,
    tstr_SettingsActionsHotkeysTip,
    tstr_SettingsActionsHotkeysAdd,
    tstr_SettingsActionsHotkeysRemove,
    tstr_SettingsActionsTableHeaderAction,
    tstr_SettingsActionsTableHeaderShortcut,
    tstr_SettingsActionsTableHeaderHotkey,
    tstr_SettingsActionsManageHeader,
    tstr_SettingsActionsManageCopyUID,
    tstr_SettingsActionsManageNew,
    tstr_SettingsActionsManageEdit,
    tstr_SettingsActionsManageDuplicate,
    tstr_SettingsActionsManageDelete,
    tstr_SettingsActionsManageDeleteConfirm,
    tstr_SettingsActionsManageDuplicatedName,                   //%NAME% == Action name
    tstr_SettingsActionsEditHeader,
    tstr_SettingsActionsEditName,
    tstr_SettingsActionsEditNameTranslatedTip,
    tstr_SettingsActionsEditTarget,
    tstr_SettingsActionsEditTargetDefault,
    tstr_SettingsActionsEditTargetDefaultTip,
    tstr_SettingsActionsEditTargetUseTags,
    tstr_SettingsActionsEditTargetActionTarget,
    tstr_SettingsActionsEditHeaderAppearance,
    tstr_SettingsActionsEditIcon,
    tstr_SettingsActionsEditLabel,
    tstr_SettingsActionsEditLabelTranslatedTip,
    tstr_SettingsActionsEditHeaderCommands,
    tstr_SettingsActionsEditNameNew,
    tstr_SettingsActionsEditCommandAdd,
    tstr_SettingsActionsEditCommandDelete,
    tstr_SettingsActionsEditCommandDeleteConfirm,
    tstr_SettingsActionsEditCommandType,
    tstr_SettingsActionsEditCommandTypeNone,
    tstr_SettingsActionsEditCommandTypeKey,
    tstr_SettingsActionsEditCommandTypeMousePos,
    tstr_SettingsActionsEditCommandTypeString,
    tstr_SettingsActionsEditCommandTypeLaunchApp,
    tstr_SettingsActionsEditCommandTypeShowKeyboard,
    tstr_SettingsActionsEditCommandTypeCropActiveWindow,
    tstr_SettingsActionsEditCommandTypeShowOverlay,
    tstr_SettingsActionsEditCommandTypeSwitchTask,
    tstr_SettingsActionsEditCommandTypeLoadOverlayProfile,
    tstr_SettingsActionsEditCommandTypeUnknown,
    tstr_SettingsActionsEditCommandVisibilityToggle,
    tstr_SettingsActionsEditCommandVisibilityShow,
    tstr_SettingsActionsEditCommandVisibilityHide,
    tstr_SettingsActionsEditCommandUndo,
    tstr_SettingsActionsEditCommandKeyCode,
    tstr_SettingsActionsEditCommandKeyToggle,
    tstr_SettingsActionsEditCommandMouseX,
    tstr_SettingsActionsEditCommandMouseY,
    tstr_SettingsActionsEditCommandMouseUseCurrent,
    tstr_SettingsActionsEditCommandString,
    tstr_SettingsActionsEditCommandPath,
    tstr_SettingsActionsEditCommandPathTip,
    tstr_SettingsActionsEditCommandArgs,
    tstr_SettingsActionsEditCommandArgsTip,
    tstr_SettingsActionsEditCommandVisibility,
    tstr_SettingsActionsEditCommandSwitchingMethod,
    tstr_SettingsActionsEditCommandSwitchingMethodSwitcher,
    tstr_SettingsActionsEditCommandSwitchingMethodFocus,
    tstr_SettingsActionsEditCommandWindow,
    tstr_SettingsActionsEditCommandWindowNone,
    tstr_SettingsActionsEditCommandCursorWarp,
    tstr_SettingsActionsEditCommandProfile,
    tstr_SettingsActionsEditCommandProfileClear,
    tstr_SettingsActionsEditCommandDescNone,
    tstr_SettingsActionsEditCommandDescKey,                     //%KEYNAME% == key name
    tstr_SettingsActionsEditCommandDescKeyToggle,               //^
    tstr_SettingsActionsEditCommandDescMousePos,                //%X% & %Y% == position coordinates
    tstr_SettingsActionsEditCommandDescString,                  //%STRING% == command string
    tstr_SettingsActionsEditCommandDescLaunchApp,               //%APP% == app path, %ARGSOPT% == tstr_SettingsActionsEditCommandDescLaunchAppArgsOpt or blank if app args are empty
    tstr_SettingsActionsEditCommandDescLaunchAppArgsOpt,        //%ARGS% == app arguments
    tstr_SettingsActionsEditCommandDescKeyboardToggle,
    tstr_SettingsActionsEditCommandDescKeyboardShow,
    tstr_SettingsActionsEditCommandDescKeyboardHide,
    tstr_SettingsActionsEditCommandDescCropWindow,
    tstr_SettingsActionsEditCommandDescOverlayToggle,           //%TAGS% == target tags
    tstr_SettingsActionsEditCommandDescOverlayShow,             //^
    tstr_SettingsActionsEditCommandDescOverlayHide,             //^
    tstr_SettingsActionsEditCommandDescOverlayTargetDefault,
    tstr_SettingsActionsEditCommandDescSwitchTask,
    tstr_SettingsActionsEditCommandDescSwitchTaskWindow,        //%WINDOW% == window title string
    tstr_SettingsActionsEditCommandDescLoadOverlayProfile,      //%PROFILE% == profile name string
    tstr_SettingsActionsEditCommandDescLoadOverlayProfileAdd,   //^
    tstr_SettingsActionsEditCommandDescUnknown,
    tstr_SettingsActionsOrderHeader,
    tstr_SettingsActionsOrderButtonLabel,                       //%COUNT% == Action count
    tstr_SettingsActionsOrderButtonLabelSingular,               //^
    tstr_SettingsActionsOrderNoActions,
    tstr_SettingsActionsOrderAdd,
    tstr_SettingsActionsOrderRemove,
    tstr_SettingsActionsAddSelectorHeader,
    tstr_SettingsActionsAddSelectorAdd,
    tstr_SettingsKeyboardLayout,
    tstr_SettingsKeyboardSize,
    tstr_SettingsKeyboardBehavior,
    tstr_SettingsKeyboardStickyMod,
    tstr_SettingsKeyboardKeyRepeat,
    tstr_SettingsKeyboardAutoShow,
    tstr_SettingsKeyboardAutoShowDesktopOnly,
    tstr_SettingsKeyboardAutoShowDesktop,
    tstr_SettingsKeyboardAutoShowDesktopTip,
    tstr_SettingsKeyboardAutoShowBrowser,
    tstr_SettingsKeyboardLayoutAuthor,                          //%AUTHOR% == Layout Author string
    tstr_SettingsKeyboardKeyClusters,
    tstr_SettingsKeyboardKeyClusterBase,
    tstr_SettingsKeyboardKeyClusterFunction,
    tstr_SettingsKeyboardKeyClusterNavigation,
    tstr_SettingsKeyboardKeyClusterNumpad,
    tstr_SettingsKeyboardKeyClusterExtra,
    tstr_SettingsKeyboardSwitchToEditor,
    tstr_SettingsMouseShowCursor,
    tstr_SettingsMouseShowCursorGCUnsupported,
    tstr_SettingsMouseShowCursorGCActiveWarning,
    tstr_SettingsMouseScrollSmooth,
    tstr_SettingsMouseSimulatePen,
    tstr_SettingsMouseSimulatePenUnsupported,
    tstr_SettingsMouseAllowLaserPointerOverride,
    tstr_SettingsMouseAllowLaserPointerOverrideTip,
    tstr_SettingsMouseDoubleClickAssist,
    tstr_SettingsMouseDoubleClickAssistTip,
    tstr_SettingsMouseDoubleClickAssistTipValueOff,
    tstr_SettingsMouseDoubleClickAssistTipValueAuto,
    tstr_SettingsMouseSmoothing,
    tstr_SettingsMouseSmoothingLevelNone,
    tstr_SettingsMouseSmoothingLevelVeryLow,
    tstr_SettingsMouseSmoothingLevelLow,
    tstr_SettingsMouseSmoothingLevelMedium,
    tstr_SettingsMouseSmoothingLevelHigh,
    tstr_SettingsMouseSmoothingLevelVeryHigh,
    tstr_SettingsLaserPointerTip,
    tstr_SettingsLaserPointerBlockInput,
    tstr_SettingsLaserPointerAutoToggleDistance,
    tstr_SettingsLaserPointerAutoToggleDistanceValueOff,
    tstr_SettingsLaserPointerHMDPointer,
    tstr_SettingsLaserPointerHMDPointerTableHeaderInputAction,
    tstr_SettingsLaserPointerHMDPointerTableHeaderBinding,
    tstr_SettingsLaserPointerHMDPointerTableBindingToggle,
    tstr_SettingsLaserPointerHMDPointerTableBindingLeft,
    tstr_SettingsLaserPointerHMDPointerTableBindingRight,
    tstr_SettingsLaserPointerHMDPointerTableBindingMiddle,
    tstr_SettingsWindowOverlaysAutoFocus,
    tstr_SettingsWindowOverlaysKeepOnScreen,
    tstr_SettingsWindowOverlaysKeepOnScreenTip,
    tstr_SettingsWindowOverlaysAutoSizeOverlay,
    tstr_SettingsWindowOverlaysFocusSceneApp,
    tstr_SettingsWindowOverlaysOnWindowDrag,
    tstr_SettingsWindowOverlaysOnWindowDragDoNothing,
    tstr_SettingsWindowOverlaysOnWindowDragBlock,
    tstr_SettingsWindowOverlaysOnWindowDragOverlay,
    tstr_SettingsWindowOverlaysOnCaptureLoss,
    tstr_SettingsWindowOverlaysOnCaptureLossTip,
    tstr_SettingsWindowOverlaysOnCaptureLossDoNothing,
    tstr_SettingsWindowOverlaysOnCaptureLossHide,
    tstr_SettingsWindowOverlaysOnCaptureLossRemove,
    tstr_SettingsBrowserMaxFrameRate,
    tstr_SettingsBrowserMaxFrameRateOverrideOff,
    tstr_SettingsBrowserContentBlocker,
    tstr_SettingsBrowserContentBlockerTip,
    tstr_SettingsBrowserContentBlockerListCount,          //%LISTCOUNT% == List count
    tstr_SettingsBrowserContentBlockerListCountSingular,  //^
    tstr_SettingsPerformanceUpdateLimiter,
    tstr_SettingsPerformanceUpdateLimiterMode,
    tstr_SettingsPerformanceUpdateLimiterModeOff,
    tstr_SettingsPerformanceUpdateLimiterModeMS,
    tstr_SettingsPerformanceUpdateLimiterModeFPS,
    tstr_SettingsPerformanceUpdateLimiterModeOffOverride,
    tstr_SettingsPerformanceUpdateLimiterModeMSTip,
    tstr_SettingsPerformanceUpdateLimiterFPSValue,        //%FPS% == FPS value
    tstr_SettingsPerformanceUpdateLimiterOverride,
    tstr_SettingsPerformanceUpdateLimiterOverrideTip,
    tstr_SettingsPerformanceUpdateLimiterModeOverride,
    tstr_SettingsPerformanceRapidUpdates,
    tstr_SettingsPerformanceRapidUpdatesTip,
    tstr_SettingsPerformanceSingleDesktopMirror,
    tstr_SettingsPerformanceSingleDesktopMirrorTip,
    tstr_SettingsPerformanceShowFPS,
    tstr_SettingsWarningsHidden,
    tstr_SettingsWarningsReset,
    tstr_SettingsStartupAutoLaunch,
    tstr_SettingsStartupSteamDisable,
    tstr_SettingsStartupSteamDisableTip,
    tstr_SettingsTroubleshootingRestart,
    tstr_SettingsTroubleshootingRestartSteam,
    tstr_SettingsTroubleshootingRestartDesktop,
    tstr_SettingsTroubleshootingElevatedModeEnter,
    tstr_SettingsTroubleshootingElevatedModeLeave,
    tstr_SettingsTroubleshootingSettingsReset,
    tstr_SettingsTroubleshootingSettingsResetConfirmDescription,
    tstr_SettingsTroubleshootingSettingsResetConfirmButton,
    tstr_SettingsTroubleshootingSettingsResetConfirmElementOverlays,
    tstr_SettingsTroubleshootingSettingsResetConfirmElementLegacyFiles,
    tstr_SettingsTroubleshootingSettingsResetShowQuickStart,
    tstr_KeyboardWindowTitle,
    tstr_KeyboardWindowTitleSettings,
    tstr_KeyboardWindowTitleOverlay,                      //%OVERLAYNAME% == input target overlay name
    tstr_KeyboardWindowTitleOverlayUnknown,
    tstr_KeyboardShortcutsCut,
    tstr_KeyboardShortcutsCopy,
    tstr_KeyboardShortcutsPaste,
    tstr_OvrlPropsCatPosition,
    tstr_OvrlPropsCatAppearance,
    tstr_OvrlPropsCatCapture,
    tstr_OvrlPropsCatPerformanceMonitor,
    tstr_OvrlPropsCatBrowser,
    tstr_OvrlPropsCatAdvanced,
    tstr_OvrlPropsCatPerformance,
    tstr_OvrlPropsCatInterface,
    tstr_OvrlPropsPositionOrigin,
    tstr_OvrlPropsPositionOriginRoom,
    tstr_OvrlPropsPositionOriginHMDXY,
    tstr_OvrlPropsPositionOriginSeatedSpace,
    tstr_OvrlPropsPositionOriginDashboard,
    tstr_OvrlPropsPositionOriginHMD,
    tstr_OvrlPropsPositionOriginControllerL,
    tstr_OvrlPropsPositionOriginControllerR,
    tstr_OvrlPropsPositionOriginTracker1,
    tstr_OvrlPropsPositionOriginTheaterScreen,
    tstr_OvrlPropsPositionOriginConfigHMDXYTurning,
    tstr_OvrlPropsPositionOriginConfigTheaterScreenEnter,
    tstr_OvrlPropsPositionOriginConfigTheaterScreenLeave,
    tstr_OvrlPropsPositionOriginTheaterScreenTip,
    tstr_OvrlPropsPositionDispMode,
    tstr_OvrlPropsPositionDispModeAlways,
    tstr_OvrlPropsPositionDispModeDashboard,
    tstr_OvrlPropsPositionDispModeScene,
    tstr_OvrlPropsPositionDispModeDPlus,
    tstr_OvrlPropsPositionPos,
    tstr_OvrlPropsPositionPosTip,
    tstr_OvrlPropsPositionChange,
    tstr_OvrlPropsPositionReset,
    tstr_OvrlPropsPositionLock,
    tstr_OvrlPropsPositionChangeHeader,
    tstr_OvrlPropsPositionChangeHelp,
    tstr_OvrlPropsPositionChangeHelpDesktop,
    tstr_OvrlPropsPositionChangeManualAdjustment,
    tstr_OvrlPropsPositionChangeMove,
    tstr_OvrlPropsPositionChangeRotate,
    tstr_OvrlPropsPositionChangeForward,
    tstr_OvrlPropsPositionChangeBackward,
    tstr_OvrlPropsPositionChangeRollCW,
    tstr_OvrlPropsPositionChangeRollCCW,
    tstr_OvrlPropsPositionChangeLookAt,
    tstr_OvrlPropsPositionChangeDragButton,
    tstr_OvrlPropsPositionChangeOffset,
    tstr_OvrlPropsPositionChangeOffsetUpDown,
    tstr_OvrlPropsPositionChangeOffsetRightLeft,
    tstr_OvrlPropsPositionChangeOffsetForwardBackward,
    tstr_OvrlPropsPositionChangeDragSettings,
    tstr_OvrlPropsPositionChangeDragSettingsForceUpright,
    tstr_OvrlPropsPositionChangeDragSettingsAutoDocking,
    tstr_OvrlPropsPositionChangeDragSettingsForceDistance,
    tstr_OvrlPropsPositionChangeDragSettingsForceDistanceShape,
    tstr_OvrlPropsPositionChangeDragSettingsForceDistanceShapeSphere,
    tstr_OvrlPropsPositionChangeDragSettingsForceDistanceShapeCylinder,
    tstr_OvrlPropsPositionChangeDragSettingsForceDistanceAutoCurve,
    tstr_OvrlPropsPositionChangeDragSettingsForceDistanceAutoTilt,
    tstr_OvrlPropsPositionChangeDragSettingsSnapPosition,
    tstr_OvrlPropsAppearanceWidth,
    tstr_OvrlPropsAppearanceCurve,
    tstr_OvrlPropsAppearanceOpacity,
    tstr_OvrlPropsAppearanceBrightness,
    tstr_OvrlPropsAppearanceCrop,
    tstr_OvrlPropsAppearanceCropValueMax,
    tstr_OvrlPropsCrop,
    tstr_OvrlPropsCropHelp,
    tstr_OvrlPropsCropManualAdjust,
    tstr_OvrlPropsCropInvalidTip,
    tstr_OvrlPropsCropX,
    tstr_OvrlPropsCropY,
    tstr_OvrlPropsCropWidth,
    tstr_OvrlPropsCropHeight,
    tstr_OvrlPropsCropToWindow,
    tstr_OvrlPropsCaptureMethod,
    tstr_OvrlPropsCaptureMethodDup,
    tstr_OvrlPropsCaptureMethodGC,
    tstr_OvrlPropsCaptureMethodGCUnsupportedTip,
    tstr_OvrlPropsCaptureMethodGCUnsupportedPartialTip,
    tstr_OvrlPropsCaptureSource,
    tstr_OvrlPropsCaptureGCSource,
    tstr_OvrlPropsCaptureSourceUnknownWarning,
    tstr_OvrlPropsCaptureGCStrictMatching,
    tstr_OvrlPropsCaptureGCStrictMatchingTip,
    tstr_OvrlPropsPerfMonDesktopModeTip,
    tstr_OvrlPropsPerfMonGlobalTip,
    tstr_OvrlPropsPerfMonStyle,
    tstr_OvrlPropsPerfMonStyleCompact,
    tstr_OvrlPropsPerfMonStyleLarge,
    tstr_OvrlPropsPerfMonShowCPU,
    tstr_OvrlPropsPerfMonShowGPU,
    tstr_OvrlPropsPerfMonShowGraphs,
    tstr_OvrlPropsPerfMonShowFrameStats,
    tstr_OvrlPropsPerfMonShowTime,
    tstr_OvrlPropsPerfMonShowBattery,
    tstr_OvrlPropsPerfMonShowTrackerBattery,
    tstr_OvrlPropsPerfMonShowViveWirelessTemp,
    tstr_OvrlPropsPerfMonDisableGPUCounter,
    tstr_OvrlPropsPerfMonDisableGPUCounterTip,
    tstr_OvrlPropsPerfMonResetValues,
    tstr_OvrlPropsBrowserNotAvailableTip,
    tstr_OvrlPropsBrowserCloned,
    tstr_OvrlPropsBrowserClonedTip,             //%OVERLAYNAME% == duplication source overlay name
    tstr_OvrlPropsBrowserClonedConvert,
    tstr_OvrlPropsBrowserURL,
    tstr_OvrlPropsBrowserURLHint,
    tstr_OvrlPropsBrowserGo,
    tstr_OvrlPropsBrowserRestore,
    tstr_OvrlPropsBrowserWidth,
    tstr_OvrlPropsBrowserHeight,
    tstr_OvrlPropsBrowserZoom,
    tstr_OvrlPropsBrowserAllowTransparency,
    tstr_OvrlPropsBrowserAllowTransparencyTip,
    tstr_OvrlPropsBrowserRecreateContext,
    tstr_OvrlPropsBrowserRecreateContextTip,
    tstr_OvrlPropsAdvanced3D,
    tstr_OvrlPropsAdvancedHSBS,
    tstr_OvrlPropsAdvancedSBS,
    tstr_OvrlPropsAdvancedHOU,
    tstr_OvrlPropsAdvancedOU,
    tstr_OvrlPropsAdvanced3DSwap,
    tstr_OvrlPropsAdvancedGazeFade,
    tstr_OvrlPropsAdvancedGazeFadeAuto,
    tstr_OvrlPropsAdvancedGazeFadeDistance,
    tstr_OvrlPropsAdvancedGazeFadeDistanceValueInf,
    tstr_OvrlPropsAdvancedGazeFadeSensitivity,
    tstr_OvrlPropsAdvancedGazeFadeOpacity,
    tstr_OvrlPropsAdvancedInput,
    tstr_OvrlPropsAdvancedInputInGame,
    tstr_OvrlPropsAdvancedInputFloatingUI,
    tstr_OvrlPropsAdvancedOverlayTags,
    tstr_OvrlPropsAdvancedOverlayTagsTip,
    tstr_OvrlPropsPerformanceInvisibleUpdate,
    tstr_OvrlPropsPerformanceInvisibleUpdateTip,
    tstr_OvrlPropsInterfaceOverlayName,
    tstr_OvrlPropsInterfaceOverlayNameAuto,
    tstr_OvrlPropsInterfaceActionOrderCustom,
    tstr_OvrlPropsInterfaceDesktopButtons,
    tstr_OvrlPropsInterfaceExtraButtons,
    tstr_OverlayBarOvrlHide,
    tstr_OverlayBarOvrlShow,
    tstr_OverlayBarOvrlClone,
    tstr_OverlayBarOvrlRemove,
    tstr_OverlayBarOvrlRemoveConfirm,
    tstr_OverlayBarOvrlProperties,
    tstr_OverlayBarOvrlAddWindow,
    tstr_OverlayBarTooltipOvrlAdd,
    tstr_OverlayBarTooltipSettings,
    tstr_OverlayBarTooltipResetHold,
    tstr_FloatingUIHideOverlayTip,
    tstr_FloatingUIHideOverlayHoldTip,
    tstr_FloatingUIDragModeEnableTip,
    tstr_FloatingUIDragModeDisableTip,
    tstr_FloatingUIDragModeHoldLockTip,
    tstr_FloatingUIDragModeHoldUnlockTip,
    tstr_FloatingUIWindowAddTip,
    tstr_FloatingUIActionBarShowTip,
    tstr_FloatingUIActionBarHideTip,
    tstr_FloatingUIBrowserGoBackTip,
    tstr_FloatingUIBrowserGoForwardTip,
    tstr_FloatingUIBrowserRefreshTip,
    tstr_FloatingUIBrowserStopTip,
    tstr_FloatingUIActionBarDesktopPrev,
    tstr_FloatingUIActionBarDesktopNext,
    tstr_FloatingUIActionBarEmpty,
    tstr_ActionNone,
    tstr_ActionKeyboardShow,
    tstr_ActionKeyboardHide,
    tstr_DefActionShowKeyboard,
    tstr_DefActionActiveWindowCrop,
    tstr_DefActionActiveWindowCropLabel,
    tstr_DefActionSwitchTask,
    tstr_DefActionToggleOverlays,
    tstr_DefActionToggleOverlaysLabel,
    tstr_DefActionMiddleMouse,
    tstr_DefActionMiddleMouseLabel,
    tstr_DefActionBackMouse,
    tstr_DefActionBackMouseLabel,
    tstr_DefActionReadMe,
    tstr_DefActionReadMeLabel,
    tstr_DefActionDashboardToggle,
    tstr_DefActionDashboardToggleLabel,
    tstr_PerformanceMonitorCPU,
    tstr_PerformanceMonitorGPU,
    tstr_PerformanceMonitorRAM,
    tstr_PerformanceMonitorVRAM,
    tstr_PerformanceMonitorFrameTime,
    tstr_PerformanceMonitorLoad,
    tstr_PerformanceMonitorFPS,
    tstr_PerformanceMonitorFPSAverage,
    tstr_PerformanceMonitorReprojectionRatio,
    tstr_PerformanceMonitorDroppedFrames,
    tstr_PerformanceMonitorBatteryLeft,
    tstr_PerformanceMonitorBatteryRight,
    tstr_PerformanceMonitorBatteryHMD,
    tstr_PerformanceMonitorBatteryTracker,
    tstr_PerformanceMonitorBatteryDisconnected,
    tstr_PerformanceMonitorViveWirelessTempNotAvailable,
    tstr_PerformanceMonitorCompactCPU,
    tstr_PerformanceMonitorCompactGPU,
    tstr_PerformanceMonitorCompactFPS,
    tstr_PerformanceMonitorCompactFPSAverage,
    tstr_PerformanceMonitorCompactReprojectionRatio,
    tstr_PerformanceMonitorCompactDroppedFrames,
    tstr_PerformanceMonitorCompactBattery,
    tstr_PerformanceMonitorCompactBatteryLeft,
    tstr_PerformanceMonitorCompactBatteryRight,
    tstr_PerformanceMonitorCompactBatteryHMD,
    tstr_PerformanceMonitorCompactBatteryTracker,
    tstr_PerformanceMonitorCompactBatteryDisconnected,
    tstr_PerformanceMonitorCompactViveWirelessTempNotAvailable,
    tstr_PerformanceMonitorEmpty,
    tstr_AuxUIDragHintDocking,
    tstr_AuxUIDragHintUndocking,
    tstr_AuxUIDragHintOvrlLocked,
    tstr_AuxUIDragHintOvrlTheaterScreenBlocked,
    tstr_AuxUIGazeFadeAutoHint,           //%SECONDS% == Countdown seconds
    tstr_AuxUIGazeFadeAutoHintSingular,   //^
    tstr_AuxUIQuickStartWelcomeHeader,
    tstr_AuxUIQuickStartWelcomeBody,
    tstr_AuxUIQuickStartOverlaysHeader,
    tstr_AuxUIQuickStartOverlaysBody,
    tstr_AuxUIQuickStartOverlaysBody2,
    tstr_AuxUIQuickStartOverlayPropertiesHeader,
    tstr_AuxUIQuickStartOverlayPropertiesBody,
    tstr_AuxUIQuickStartOverlayPropertiesBody2,
    tstr_AuxUIQuickStartSettingsHeader,
    tstr_AuxUIQuickStartSettingsBody,
    tstr_AuxUIQuickStartProfilesHeader,
    tstr_AuxUIQuickStartProfilesBody,
    tstr_AuxUIQuickStartActionsHeader,
    tstr_AuxUIQuickStartActionsBody,
    tstr_AuxUIQuickStartActionsBody2,
    tstr_AuxUIQuickStartOverlayTagsHeader,
    tstr_AuxUIQuickStartOverlayTagsBody,
    tstr_AuxUIQuickStartSettingsEndBody,
    tstr_AuxUIQuickStartFloatingUIHeader,
    tstr_AuxUIQuickStartFloatingUIBody,
    tstr_AuxUIQuickStartDesktopModeHeader,
    tstr_AuxUIQuickStartDesktopModeBody,
    tstr_AuxUIQuickStartEndHeader,
    tstr_AuxUIQuickStartEndBody,
    tstr_AuxUIQuickStartButtonNext,
    tstr_AuxUIQuickStartButtonPrev,
    tstr_AuxUIQuickStartButtonClose,
    tstr_DesktopModeCatTools,
    tstr_DesktopModeCatOverlays,
    tstr_DesktopModeToolSettings,
    tstr_DesktopModeToolActions,
    tstr_DesktopModeOverlayListAdd,
    tstr_DesktopModePageAddWindowOverlayTitle,
    tstr_DesktopModePageAddWindowOverlayHeader,
    tstr_KeyboardEditorKeyListTitle,
    tstr_KeyboardEditorKeyListTabContextReplace,
    tstr_KeyboardEditorKeyListTabContextClear,
    tstr_KeyboardEditorKeyListRow,       //%ID% == Row ID
    tstr_KeyboardEditorKeyListSpacing,
    tstr_KeyboardEditorKeyListKeyAdd,
    tstr_KeyboardEditorKeyListKeyDuplicate,
    tstr_KeyboardEditorKeyListKeyRemove,
    tstr_KeyboardEditorKeyPropertiesTitle,
    tstr_KeyboardEditorKeyPropertiesNoSelection,
    tstr_KeyboardEditorKeyPropertiesType,
    tstr_KeyboardEditorKeyPropertiesTypeBlank,
    tstr_KeyboardEditorKeyPropertiesTypeVirtualKey,
    tstr_KeyboardEditorKeyPropertiesTypeVirtualKeyToggle,
    tstr_KeyboardEditorKeyPropertiesTypeVirtualKeyIsoEnter,
    tstr_KeyboardEditorKeyPropertiesTypeString,
    tstr_KeyboardEditorKeyPropertiesTypeSublayoutToggle,
    tstr_KeyboardEditorKeyPropertiesTypeAction,
    tstr_KeyboardEditorKeyPropertiesTypeVirtualKeyIsoEnterTip,
    tstr_KeyboardEditorKeyPropertiesTypeStringTip,
    tstr_KeyboardEditorKeyPropertiesSize,
    tstr_KeyboardEditorKeyPropertiesLabel,
    tstr_KeyboardEditorKeyPropertiesKeyCode,
    tstr_KeyboardEditorKeyPropertiesString,
    tstr_KeyboardEditorKeyPropertiesSublayout,
    tstr_KeyboardEditorKeyPropertiesAction,
    tstr_KeyboardEditorKeyPropertiesCluster,
    tstr_KeyboardEditorKeyPropertiesClusterTip,
    tstr_KeyboardEditorKeyPropertiesBlockModifiers,
    tstr_KeyboardEditorKeyPropertiesBlockModifiersTip,
    tstr_KeyboardEditorKeyPropertiesNoRepeat,
    tstr_KeyboardEditorKeyPropertiesNoRepeatTip,
    tstr_KeyboardEditorMetadataTitle,
    tstr_KeyboardEditorMetadataName,
    tstr_KeyboardEditorMetadataAuthor,
    tstr_KeyboardEditorMetadataHasAltGr,
    tstr_KeyboardEditorMetadataHasAltGrTip,
    tstr_KeyboardEditorMetadataClusterPreview,
    tstr_KeyboardEditorMetadataSave,
    tstr_KeyboardEditorMetadataLoad,
    tstr_KeyboardEditorMetadataSavePopupTitle,
    tstr_KeyboardEditorMetadataSavePopupFilename,
    tstr_KeyboardEditorMetadataSavePopupFilenameBlankTip,
    tstr_KeyboardEditorMetadataSavePopupConfirm,
    tstr_KeyboardEditorMetadataSavePopupConfirmError,
    tstr_KeyboardEditorMetadataLoadPopupTitle,
    tstr_KeyboardEditorMetadataLoadPopupConfirm,
    tstr_KeyboardEditorPreviewTitle,
    tstr_KeyboardEditorSublayoutBase,
    tstr_KeyboardEditorSublayoutShift,
    tstr_KeyboardEditorSublayoutAltGr,
    tstr_KeyboardEditorSublayoutAux,
    tstr_DialogOk,
    tstr_DialogCancel,
    tstr_DialogDone,
    tstr_DialogUndo,
    tstr_DialogRedo,
    tstr_DialogColorPickerHeader,
    tstr_DialogColorPickerCurrent,
    tstr_DialogColorPickerOriginal,
    tstr_DialogProfilePickerHeader,
    tstr_DialogProfilePickerNone,
    tstr_DialogActionPickerHeader,
    tstr_DialogActionPickerEmpty,
    tstr_DialogIconPickerHeader,
    tstr_DialogIconPickerHeaderTip,
    tstr_DialogIconPickerNone,
    tstr_DialogKeyCodePickerHeader,
    tstr_DialogKeyCodePickerHeaderHotkey,
    tstr_DialogKeyCodePickerModifiers,
    tstr_DialogKeyCodePickerKeyCode,
    tstr_DialogKeyCodePickerKeyCodeHint,
    tstr_DialogKeyCodePickerKeyCodeNone,
    tstr_DialogKeyCodePickerFromInput,
    tstr_DialogKeyCodePickerFromInputPopup,
    tstr_DialogKeyCodePickerFromInputPopupNoMouse,
    tstr_DialogInputTagsHint,
    tstr_SourceDesktopAll,
    tstr_SourceDesktopID,   //%ID% == Desktop ID
    tstr_SourceWinRTNone,
    tstr_SourceWinRTUnknown,
    tstr_SourceWinRTClosed,
    tstr_SourcePerformanceMonitor,
    tstr_SourceBrowser,
    tstr_SourceBrowserNoPage,
    tstr_NotificationIconRestoreVR,
    tstr_NotificationIconOpenOnDesktop,
    tstr_NotificationIconQuit,
    tstr_NotificationInitialStartupTitleVR,
    tstr_NotificationInitialStartupTitleDesktop,
    tstr_NotificationInitialStartupMessage,
    tstr_BrowserErrorPageTitle,
    tstr_BrowserErrorPageHeading,
    tstr_BrowserErrorPageMessage,
    tstr_MAX,
    tstr_NONE = tstr_MAX //Don't pass this into GetString()
};

class TranslationManager
{
    public:
        struct ListEntry
        {
            std::string FileName;
            std::string ListName;
        };

    private:
        static const char* s_StringIDNames[tstr_MAX];
        std::string m_Strings[tstr_MAX];

        //Precomputed strings, updated after loading different translations
        std::vector<std::string> m_StringsDesktopID;
        std::vector<std::string> m_StringsFPSLimit;

        std::string m_CurrentTranslationName;
        std::string m_CurrentTranslationAuthor;
        bool m_IsCurrentTranslationComplete;

    public:
        TranslationManager();
        static TranslationManager& Get();
        static const char* GetString(TRMGRStrID str_id);      //Strings are not sanitized in any way, so probably a bad idea to feed straight into ImGui functions calling printf
        static TRMGRStrID GetStringID(const char* str);       //May return tstr_NONE
        static std::string GetTranslationNameFromFile(const std::string& filename);
        static std::vector<TranslationManager::ListEntry> GetTranslationList();

        void LoadTranslationFromFile(const std::string& filename);

        const std::string& GetCurrentTranslationName() const;
        const std::string& GetCurrentTranslationAuthor() const;
        bool IsCurrentTranslationComplete() const;
        void AddStringsToFontBuilder(ImFontGlyphRangesBuilder& builder) const;

        const char* GetDesktopIDString(int desktop_id);
        const char* GetFPSLimitString(int fps_limit_id);
};