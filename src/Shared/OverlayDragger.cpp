#include "OverlayDragger.h"

#ifdef DPLUS_UI
    #include "UIManager.h"
#else
    #include "OutputManager.h"
#endif

#include "OverlayManager.h"
#include "InterprocessMessaging.h"
#include "Util.h"
#include "OpenVRExt.h"
#include <SnapRotationUtil.h>

OverlayDragger::OverlayDragger() : 
    m_DragModeDeviceID(-1),
    m_DragModeOverlayID(k_ulOverlayID_None),
    m_DragModeOverlayHandle(vr::k_ulOverlayHandleInvalid),
    m_DragModeOverlayOrigin(ovrl_origin_room),
    m_DragModeMaxWidth(FLT_MAX),
    m_DragModeSnappedExtraWidth(0.0f),
    m_DragGestureActive(false),
    m_DragGestureScaleDistanceStart(0.0f),
    m_DragGestureScaleWidthStart(0.0f),
    m_DragGestureScaleDistanceLast(0.0f),
    m_AbsoluteModeActive(false),
    m_AbsoluteModeOffsetForward(0.0f),
    m_DashboardHMD_Y(-100.0f)
{
    m_DashboardMatLast.zero();
}

void OverlayDragger::DragStartBase(bool is_gesture_drag)
{
    if ( (IsDragActive()) || (IsDragGestureActive()) )
        return;

    //This is also used by DragGestureStart() (with is_gesture_drag = true), but only to convert between overlay origins.
    //Doesn't need calls to the other DragUpdate() or DragFinish() functions in that case
    vr::TrackedDeviceIndex_t device_index = ConfigManager::Get().GetPrimaryLaserPointerDevice();

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, vr::IVRSystemEx::GetTimeNowToPhotons(), poses, vr::k_unMaxTrackedDeviceCount);

    //We have no dashboard device, but something still started a drag, eh? This happens when the dashboard is closed but the overlays are still interactive
    //There doesn't seem to be a way to get around this, so we guess by checking which of the two hand controllers are currently pointing at the overlay
    //Works for most cases at least
    if (device_index == vr::k_unTrackedDeviceIndexInvalid)
    {
        device_index = vr::IVROverlayEx::FindPointerDeviceForOverlay(m_DragModeOverlayHandle);

        //Still nothing, try the config hint
        if (device_index == vr::k_unTrackedDeviceIndexInvalid)
        {
            device_index = (vr::TrackedDeviceIndex_t)ConfigManager::GetValue(configid_int_state_laser_pointer_device_hint);
        }
    }

    //Use HMD as device when the tracked device will never have a valid pose (e.g. gamepads)
    if ((device_index != vr::k_unTrackedDeviceIndexInvalid) && (vr::VRSystem()->GetBoolTrackedDeviceProperty(device_index, vr::Prop_NeverTracked_Bool)) )
    {
        device_index = vr::k_unTrackedDeviceIndex_Hmd;
    }

    if ( (device_index < vr::k_unMaxTrackedDeviceCount) && (poses[device_index].bPoseIsValid) )
    {
        if (!is_gesture_drag)
        {
            m_DragModeDeviceID = device_index;
        }

        m_DragModeMatrixSourceStart = poses[device_index].mDeviceToAbsoluteTracking;

        switch (m_DragModeOverlayOrigin)
        {
            case ovrl_origin_hmd:
            {
                if (poses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
                {
                    vr::VROverlay()->SetOverlayTransformAbsolute(m_DragModeOverlayHandle, vr::TrackingUniverseStanding, &poses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
                }
                break;
            }
            case ovrl_origin_right_hand:
            {
                vr::TrackedDeviceIndex_t index_right_hand = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);

                if ( (index_right_hand != vr::k_unTrackedDeviceIndexInvalid) && (poses[index_right_hand].bPoseIsValid) )
                {
                    vr::VROverlay()->SetOverlayTransformAbsolute(m_DragModeOverlayHandle, vr::TrackingUniverseStanding, &poses[index_right_hand].mDeviceToAbsoluteTracking);
                }
                break;
            }
            case ovrl_origin_left_hand:
            {
                vr::TrackedDeviceIndex_t index_left_hand = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);

                if ( (index_left_hand != vr::k_unTrackedDeviceIndexInvalid) && (poses[index_left_hand].bPoseIsValid) )
                {
                    vr::VROverlay()->SetOverlayTransformAbsolute(m_DragModeOverlayHandle, vr::TrackingUniverseStanding, &poses[index_left_hand].mDeviceToAbsoluteTracking);
                }
                break;
            }
            case ovrl_origin_aux:
            {
                vr::TrackedDeviceIndex_t index_tracker = vr::IVRSystemEx::GetFirstVRTracker();

                if ( (index_tracker != vr::k_unTrackedDeviceIndexInvalid) && (poses[index_tracker].bPoseIsValid) )
                {
                    vr::VROverlay()->SetOverlayTransformAbsolute(m_DragModeOverlayHandle, vr::TrackingUniverseStanding, &poses[index_tracker].mDeviceToAbsoluteTracking);
                }
                break;
            }
        }

        vr::HmdMatrix34_t transform_target;
        vr::TrackingUniverseOrigin origin;
        vr::VROverlay()->GetOverlayTransformAbsolute(m_DragModeOverlayHandle, &origin, &transform_target);
        m_DragModeMatrixTargetStart   = transform_target;
        m_DragModeMatrixTargetCurrent = m_DragModeMatrixTargetStart;
    }
    else
    {
        //No drag started, reset state
        m_DragModeOverlayID     = k_ulOverlayID_None;
        m_DragModeOverlayHandle = vr::k_ulOverlayHandleInvalid;
    }
}

void OverlayDragger::DragGestureStartBase()
{
    if ( (IsDragActive()) || (IsDragGestureActive()) )
        return;

    DragStartBase(true); //Call the other drag start function to convert the overlay transform to absolute. This doesn't actually start the normal drag

    DragGestureUpdate();

    m_DragGestureScaleDistanceStart = m_DragGestureScaleDistanceLast;

    if (m_DragModeOverlayID != k_ulOverlayID_None)
    {
        m_DragGestureScaleWidthStart = OverlayManager::Get().GetConfigData(m_DragModeOverlayID).ConfigFloat[configid_float_overlay_width];
    }
    else
    {
        vr::VROverlay()->GetOverlayWidthInMeters(m_DragModeOverlayHandle, &m_DragGestureScaleWidthStart);
    }

    m_DragGestureActive = true;
}

void OverlayDragger::TransformForceUpright(Matrix4& transform) const
{
    //Based off of ComputeHMDFacingTransform()... might not be the best way to do it, but it works.
    static const Vector3 up = {0.0f, 1.0f, 0.0f};

    Matrix4 matrix_temp  = transform;
    Vector3 ovrl_start   = matrix_temp.translate_relative(0.0f, 0.0f, -0.001f).getTranslation();
    Vector3 forward_temp = (ovrl_start - transform.getTranslation()).normalize();
    Vector3 right        = forward_temp.cross(up).normalize();
    Vector3 forward      = up.cross(right).normalize();

    Matrix4 mat_upright(right, up, forward * -1.0f);
    mat_upright.setTranslation(ovrl_start);

    transform = mat_upright;
}

void OverlayDragger::TransformForceDistance(Matrix4& transform, Vector3 reference_pos, float distance, bool use_cylinder_shape, bool auto_tilt) const
{
    //Match origin y-position to the overlay's to achieve cylindrical position (acts as sphere otherwise)
    if (use_cylinder_shape)
        reference_pos.y = transform.getTranslation().y;

    Matrix4 matrix_lookat = transform;
    float distance_to_reference = matrix_lookat.getTranslation().distance(reference_pos);

    //Use up-vector multiplied by rotation matrix to avoid locking at near-up transforms
    Vector3 up = matrix_lookat * Vector3(0.0f, 1.0f, 0.0f);
    vr::IVRSystemEx::TransformLookAt(matrix_lookat, reference_pos, up);
    matrix_lookat.translate_relative(0.0f, 0.0f, distance_to_reference - distance);

    if (auto_tilt)
    {
        //Transfer scale from original transform before replacing it with the lookat one
        Vector3 row_1(transform[0], transform[1], transform[2]);
        float scale_x = row_1.length(); //Scaling is always uniform so we just check the x-axis

        Vector3 pos = matrix_lookat.getTranslation();
        matrix_lookat.setTranslation({0.0f, 0.0f, 0.0f});
        matrix_lookat.scale(scale_x);
        matrix_lookat.setTranslation(pos);

        transform = matrix_lookat;
    }
    else
    {
        transform.setTranslation(matrix_lookat.getTranslation());
    }
}

Matrix4 OverlayDragger::GetBaseOffsetMatrix()
{
    const OverlayConfigData& data = OverlayManager::Get().GetCurrentConfigData();

    return GetBaseOffsetMatrix((OverlayOrigin)data.ConfigInt[configid_int_overlay_origin], OverlayManager::Get().GetOriginConfigFromData(data));
}

Matrix4 OverlayDragger::GetBaseOffsetMatrix(OverlayOrigin overlay_origin)
{
    return GetBaseOffsetMatrix(overlay_origin, OverlayOriginConfig());
}

Matrix4 OverlayDragger::GetBaseOffsetMatrix(OverlayOrigin overlay_origin, const OverlayOriginConfig& origin_config)
{
    Matrix4 matrix; //Identity

    vr::TrackingUniverseOrigin universe_origin = vr::TrackingUniverseStanding;

    switch (overlay_origin)
    {
        case ovrl_origin_room:
        case ovrl_origin_theater_screen:
        {
            break;
        }
        case ovrl_origin_hmd_floor:
        {
            vr::TrackedDevicePose_t poses[vr::k_unTrackedDeviceIndex_Hmd + 1];
            vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(universe_origin, vr::IVRSystemEx::GetTimeNowToPhotons(), poses, vr::k_unTrackedDeviceIndex_Hmd + 1);

            if (poses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
            {
                Matrix4 mat_pose = poses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
                Vector3 pos_offset = mat_pose.getTranslation();

                //Force HMD pose upright to have it act as turning-only base position
                if (origin_config.HMDFloorUseTurning)
                {
                    matrix = mat_pose;
                    TransformForceUpright(matrix);
                }

                pos_offset.y = 0.0f;
                matrix.setTranslation(pos_offset);
            }
            break;
        }
        case ovrl_origin_seated_universe:
        {
            matrix = vr::VRSystem()->GetSeatedZeroPoseToStandingAbsoluteTrackingPose();
            break;
        }
        case ovrl_origin_dashboard:
        {
            //Update dashboard transform if it's visible or we never set the dashboard matrix before (IsDashboardVisible() can return false while visible)
            if ( (vr::VROverlay()->IsDashboardVisible()) || (m_DashboardMatLast.isZero()) )
            {
                //This code is prone to break when Valve changes the entire dashboard once again
                vr::VROverlayHandle_t system_dashboard;
                vr::VROverlay()->FindOverlay("system.systemui", &system_dashboard);

                //Double-checking dashboard overlay visibility for the case when IsDashboardVisible() is false while it's actually visible
                if ( (system_dashboard != vr::k_ulOverlayHandleInvalid) && (vr::VROverlay()->IsOverlayVisible(system_dashboard)) )
                {
                    vr::HmdMatrix34_t matrix_overlay_system;

                    vr::HmdVector2_t overlay_system_size;
                    vr::VROverlay()->GetOverlayMouseScale(system_dashboard, &overlay_system_size); //Coordinate size should be mouse scale

                    vr::VROverlay()->GetTransformForOverlayCoordinates(system_dashboard, universe_origin, { overlay_system_size.v[0]/2.0f, 0.0f }, &matrix_overlay_system);
                    m_DashboardMatLast = matrix_overlay_system;

                    if (m_DashboardHMD_Y == -100.0f)    //If Desktop+ was started with the dashboard open, the value will still be default, so set it now
                    {
                        UpdateDashboardHMD_Y();
                    }
                }
            }

            //Adjust behavior if GamepadUI (SteamVR 2 dashboard) exists
            vr::VROverlayHandle_t handle_gamepad_ui = vr::k_ulOverlayHandleInvalid;
            vr::VROverlay()->FindOverlay("valve.steam.gamepadui.bar", &handle_gamepad_ui);

            matrix = m_DashboardMatLast;

            if (handle_gamepad_ui != vr::k_ulOverlayHandleInvalid)
            {
                //Magic number, from taking the difference of both version's dashboard origins at the same HMD position
                const Matrix4 matrix_to_old_dash( 1.14634132f,      3.725290300e-09f, -3.725290300e-09f, 0.00000000f, 
                                                  0.00000000f,      0.878148496f,      0.736854136f,     0.00000000f, 
                                                  7.45058060e-09f, -0.736854076f,      0.878148496f,     0.00000000f,
                                                 -5.96046448e-08f,  2.174717430f,      0.123533726f,     1.00000000f);

                //Move origin point roughly back to where it was in the old dashboard
                matrix = matrix * matrix_to_old_dash;

                //Move matrix towards normal dashboard overlay position
                matrix.translate_relative(0.0f, -0.57f, 0.32f);
            }
            else
            {
                //Move matrix towards normal dashboard overlay position
                matrix.translate_relative(0.0f, 1.09f, 0.0f);
            }

            break;
        }
        case ovrl_origin_hmd:
        case ovrl_origin_right_hand:
        case ovrl_origin_left_hand:
        case ovrl_origin_aux:
        {
            //This is used for the dragging only. In other cases the origin is identity, as it's attached to the controller via OpenVR
            vr::TrackedDeviceIndex_t device_index;

            switch (overlay_origin)
            {
                case ovrl_origin_hmd:        device_index = vr::k_unTrackedDeviceIndex_Hmd;                                                              break;
                case ovrl_origin_right_hand: device_index = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand); break;
                case ovrl_origin_left_hand:  device_index = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);  break;
                case ovrl_origin_aux:        device_index = vr::IVRSystemEx::GetFirstVRTracker();                                                        break;
                default:                     device_index = vr::k_unTrackedDeviceIndexInvalid;
            }

            if (device_index != vr::k_unTrackedDeviceIndexInvalid)
            {
                vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
                vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(universe_origin, vr::IVRSystemEx::GetTimeNowToPhotons(), poses, vr::k_unMaxTrackedDeviceCount);

                if (poses[device_index].bPoseIsValid)
                {
                    matrix = poses[device_index].mDeviceToAbsoluteTracking;
                }
            }
            break;
        }
        case ovrl_origin_dplus_tab:
        {
            vr::VROverlayHandle_t ovrl_handle_dplus;
            vr::VROverlay()->FindOverlay("elvissteinjr.DesktopPlusDashboard", &ovrl_handle_dplus);

            if (ovrl_handle_dplus != vr::k_ulOverlayHandleInvalid)
            {
                vr::HmdMatrix34_t matrix_dplus_tab;
                vr::TrackingUniverseOrigin origin = vr::TrackingUniverseStanding;

                vr::VROverlay()->GetTransformForOverlayCoordinates(ovrl_handle_dplus, origin, {0.5f, 0.0f}, &matrix_dplus_tab);

                matrix = matrix_dplus_tab;

                //Additional offset if GamepadUI (SteamVR 2 dashboard) exists
                vr::VROverlayHandle_t handle_gamepad_ui = vr::k_ulOverlayHandleInvalid;
                vr::VROverlay()->FindOverlay("valve.steam.gamepadui.bar", &handle_gamepad_ui);

                if (handle_gamepad_ui != vr::k_ulOverlayHandleInvalid)
                {
                    matrix.translate(0.0f, -0.05f, 0.0f);
                }
            }
            break;
        }
    }

    return matrix;
}

void OverlayDragger::ApplyDashboardScale(Matrix4& matrix)
{
    Matrix4 mat_origin = GetBaseOffsetMatrix(ovrl_origin_dplus_tab);
    Vector3 row_1(mat_origin[0], mat_origin[1], mat_origin[2]);
    float dashboard_scale = 0.469f;

    //If the dashboard scale cannot be determined yet (D+ tab hasn't been used yet), don't calculate and use the fallback value instead
    if (row_1 != Vector3(1.0f, 0.0f, 0.0f))
    {
        dashboard_scale = row_1.length();
    }

    Vector3 translation = matrix.getTranslation();
    matrix.setTranslation({0.0f, 0.0f, 0.0f});
    matrix.scale(dashboard_scale);
    matrix.setTranslation(translation);
}

void OverlayDragger::DragStart(unsigned int overlay_id)
{
    if ( (IsDragActive()) || (IsDragGestureActive()) )
        return;

    const OverlayConfigData& data = OverlayManager::Get().GetConfigData(overlay_id);

    m_DragModeDeviceID            = -1;
    m_DragModeOverlayID           = overlay_id;
    m_DragModeOverlayOrigin       = (OverlayOrigin)data.ConfigInt[configid_int_overlay_origin];
    m_DragModeOverlayOriginConfig = OverlayManager::Get().GetOriginConfigFromData(data);
    m_DragModeOverlayHandle       = data.ConfigHandle[configid_handle_overlay_state_overlay_handle];
    m_DragModeMaxWidth            = FLT_MAX;

    DragStartBase(false);
}

void OverlayDragger::DragStart(vr::VROverlayHandle_t overlay_handle, OverlayOrigin overlay_origin)
{
    if ( (IsDragActive()) || (IsDragGestureActive()) )
        return;

    m_DragModeDeviceID            = -1;
    m_DragModeOverlayID           = k_ulOverlayID_None;
    m_DragModeOverlayHandle       = overlay_handle;
    m_DragModeOverlayOrigin       = overlay_origin;
    m_DragModeOverlayOriginConfig = OverlayOriginConfig();
    m_DragModeMaxWidth            = FLT_MAX;

    DragStartBase(false);
}

void OverlayDragger::DragUpdate()
{
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, vr::IVRSystemEx::GetTimeNowToPhotons(), poses, vr::k_unMaxTrackedDeviceCount);

    if (poses[m_DragModeDeviceID].bPoseIsValid)
    {
        if (m_AbsoluteModeActive)
        {
            //Get matrices
            Matrix4 mat_device = poses[m_DragModeDeviceID].mDeviceToAbsoluteTracking;

            //Apply tip offset if controller
            mat_device = mat_device * vr::IVRSystemEx::GetControllerTipMatrix( vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(m_DragModeDeviceID) );

            //Apply forward offset
            mat_device.translate_relative(0.0f, 0.0f, -m_AbsoluteModeOffsetForward);

            m_DragModeMatrixTargetCurrent = mat_device;

            //Set transform
            vr::HmdMatrix34_t vrmat = m_DragModeMatrixTargetCurrent.toOpenVR34();
            vr::VROverlay()->SetOverlayTransformAbsolute(m_DragModeOverlayHandle, vr::TrackingUniverseStanding, &vrmat);
        }
        else
        {
            Matrix4 matrix_source_current = poses[m_DragModeDeviceID].mDeviceToAbsoluteTracking;
            Matrix4 matrix_target_new = m_DragModeMatrixTargetStart;

            Matrix4 matrix_source_start_inverse = m_DragModeMatrixSourceStart;
            matrix_source_start_inverse.invert();

            matrix_source_current = matrix_source_current * matrix_source_start_inverse;

            m_DragModeMatrixTargetCurrent = matrix_source_current * matrix_target_new;
            SnapMatrix(&m_DragModeMatrixTargetCurrent, 25.0f, true, false, true);

            //Apply drag settings if managed overlay (while most would work on UI overlays, they're more of a hindrance most of the time)
            if (m_DragModeOverlayID != k_ulOverlayID_None)
            {
                //Do axis locking if enabled
                if (ConfigManager::GetValue(configid_bool_input_drag_force_upright))
                {
                    TransformForceUpright(m_DragModeMatrixTargetCurrent);
                }

                //Snap position if enabled
                if (ConfigManager::GetValue(configid_bool_input_drag_snap_position))
                {
                    const float& snap_size = ConfigManager::GetRef(configid_float_input_drag_snap_position_size);

                    //Transform position to be multiples of snap size
                    Vector3 pos = m_DragModeMatrixTargetCurrent.getTranslation();

                    //Use center bottom if managed overlay to allow matching alignment of differently sized overlays
                    Vector3 pos_bottom_orig  = OverlayManager::Get().GetOverlayCenterBottomTransform(m_DragModeOverlayID, m_DragModeOverlayHandle).getTranslation();
                    Vector3 pos_middle = OverlayManager::Get().GetOverlayMiddleTransform(m_DragModeOverlayID, m_DragModeOverlayHandle).getTranslation();
                    Vector3 pos_bottom_offset = pos_bottom_orig - pos_middle;
                    pos += pos_bottom_offset;

                    //Snap to size
                    pos /= snap_size;
                    pos.x = roundf(pos.x);
                    pos.y = roundf(pos.y);
                    pos.z = roundf(pos.z);
                    pos *= snap_size;

                    //Get difference of bottom centered transform and apply it to the original middle-aligned position
                    pos = (pos - pos_bottom_offset);

                    m_DragModeMatrixTargetCurrent.setTranslation(pos);
                }

                //Force fixed distance if enabled
                if (ConfigManager::GetValue(configid_bool_input_drag_fixed_distance))
                {
                    const float& fixed_distance = ConfigManager::GetValue(configid_float_input_drag_fixed_distance_m);

                    TransformForceDistance(m_DragModeMatrixTargetCurrent, m_TempStandingPosition, fixed_distance, (ConfigManager::GetValue(configid_int_input_drag_fixed_distance_shape) == 1), 
                                           ConfigManager::GetValue(configid_bool_input_drag_fixed_distance_auto_tilt));

                    //Calculate curvature based on overlay width and distance if auto-curving is enabled
                    if (ConfigManager::GetValue(configid_bool_input_drag_fixed_distance_auto_curve))
                    {
                        float width  = OverlayManager::Get().GetConfigData(m_DragModeOverlayID).ConfigFloat[configid_float_overlay_width];
                        float& curve = OverlayManager::Get().GetConfigData(m_DragModeOverlayID).ConfigFloat[configid_float_overlay_curvature];

                        curve = clamp(width / (fixed_distance * 4.0f), 0.0f, 1.0f);

                        vr::VROverlay()->SetOverlayCurvature(m_DragModeOverlayHandle, curve);

                        //Sync adjusted curvature value
                        #ifdef DPLUS_UI
                            IPCManager::Get().PostConfigMessageToDashboardApp(configid_float_overlay_curvature, curve);
                        #else
                            IPCManager::Get().PostConfigMessageToUIApp(configid_float_overlay_curvature, curve);
                        #endif
                    }
                }
            }

            vr::HmdMatrix34_t vrmat = m_DragModeMatrixTargetCurrent.toOpenVR34();
            vr::VROverlay()->SetOverlayTransformAbsolute(m_DragModeOverlayHandle, vr::TrackingUniverseStanding, &vrmat);
        }
    }
}

void OverlayDragger::DragAddDistance(float distance)
{
    float overlay_width  =  1.0f;
    float overlay_height = -1.0f;

    if (m_DragModeOverlayID != k_ulOverlayID_None)
    {
        overlay_width = OverlayManager::Get().GetConfigData(m_DragModeOverlayID).ConfigFloat[configid_float_overlay_width];

        #ifndef DPLUS_UI
            if (OutputManager* outmgr = OutputManager::Get())
            {
                overlay_height = outmgr->GetOverlayHeight(m_DragModeOverlayID);
            }
        #endif
    }
    else
    {
        vr::VROverlay()->GetOverlayWidthInMeters(m_DragModeOverlayHandle, &overlay_width);

        #ifdef DPLUS_UI
            if (UIManager* uimgr = UIManager::Get())
            {
                overlay_height = uimgr->GetOverlayHeight(m_DragModeOverlayHandle);
            }
        #endif
    }

    if (overlay_height == -1.0f)
    {
        //Fallback method if above don't apply. This usually isn't used, however, as dashboard drags the managed overlays and UI the unmanaged ones.
        Vector3 pos_middle = OverlayManager::Get().GetOverlayMiddleTransform(      m_DragModeOverlayID, m_DragModeOverlayHandle).getTranslation();
        Vector3 pos_bottom = OverlayManager::Get().GetOverlayCenterBottomTransform(m_DragModeOverlayID, m_DragModeOverlayHandle).getTranslation();
        overlay_height = pos_middle.distance(pos_bottom) * 2.0f;
    }

    //Scale distance to overlay size
    distance = clamp(distance * std::max(std::min(overlay_width, overlay_height) * 0.5f, 0.1f), -0.35f, 0.35f);

    if (m_AbsoluteModeActive)
    {
        m_AbsoluteModeOffsetForward += distance * 0.5f;
        m_AbsoluteModeOffsetForward = std::max(0.01f, m_AbsoluteModeOffsetForward);
    }
    else if ( (m_DragModeOverlayID != k_ulOverlayID_None) && (ConfigManager::GetValue(configid_bool_input_drag_fixed_distance)) ) //Add to fixed distance setting instead if enabled and managed overlay
    {
        float& fixed_distance = ConfigManager::GetRef(configid_float_input_drag_fixed_distance_m);
        fixed_distance += distance * 0.5f;
        fixed_distance = std::max(0.5f, fixed_distance);

        //Sync adjusted distance value
        #ifdef DPLUS_UI
            IPCManager::Get().PostConfigMessageToDashboardApp(configid_float_input_drag_fixed_distance_m, fixed_distance);
        #else
            IPCManager::Get().PostConfigMessageToUIApp(configid_float_input_drag_fixed_distance_m, fixed_distance);
        #endif
    }
    else
    {
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, vr::IVRSystemEx::GetTimeNowToPhotons(), poses, vr::k_unMaxTrackedDeviceCount);

        if (poses[m_DragModeDeviceID].bPoseIsValid)
        {
            Matrix4 mat_drag_device = m_DragModeMatrixSourceStart;

            //Apply tip offset if possible (usually the case)
            mat_drag_device = mat_drag_device * vr::IVRSystemEx::GetControllerTipMatrix( vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(m_DragModeDeviceID) );

            //Take the drag device start orientation and the overlay's start translation and offset forward from there
            mat_drag_device.setTranslation(m_DragModeMatrixTargetStart.getTranslation());
            mat_drag_device.translate_relative(0.0f, 0.0f, distance * -0.5f);
            m_DragModeMatrixTargetStart.setTranslation(mat_drag_device.getTranslation());
        }
    }
}

float OverlayDragger::DragAddWidth(float width)
{
    if (!IsDragActive())
        return 0.0f;

    const float width_orig = width;
    width = clamp(width, -0.25f, 0.25f) + 1.0f + m_DragModeSnappedExtraWidth; //Expected range is smaller than for DragAddDistance()

    float overlay_width = 1.0f;
    float overlay_width_min = 0.05f;

    if (m_DragModeOverlayID != k_ulOverlayID_None)
    {
        const OverlayConfigData& data = OverlayManager::Get().GetConfigData(m_DragModeOverlayID);

        overlay_width = data.ConfigFloat[configid_float_overlay_width];
    }
    else
    {
        vr::VROverlay()->GetOverlayWidthInMeters(m_DragModeOverlayHandle, &overlay_width);

        overlay_width_min = 0.50f; //Usually used with ImGui window UI overlays, so use higher minimum width
    }

    const float overlay_width_orig = overlay_width;
    overlay_width *= width;

    if (overlay_width < overlay_width_min)
    {
        overlay_width = overlay_width_min;
    }

    //Snap width if snapping is enabled and managed overlay
    if ( (m_DragModeOverlayID != k_ulOverlayID_None) && (ConfigManager::GetValue(configid_bool_input_drag_snap_position)) )
    {
        const float& snap_size = ConfigManager::GetRef(configid_float_input_drag_snap_position_size);

        //Use round up/down depending on scale direction
        overlay_width  = (width > 1.0f) ? floorf(overlay_width / snap_size) : ceilf(overlay_width / snap_size);
        overlay_width *= snap_size;

        //If the snapped width had no effect, add some of it up for next time
        if (fabs(overlay_width - overlay_width_orig) < snap_size)
        {
            //Reset the extra width if the sign doesn't match anymore (direction changed)
            if (sgn(m_DragModeSnappedExtraWidth) != sgn(width_orig))
            {
                m_DragModeSnappedExtraWidth = 0.0f;
            }

            m_DragModeSnappedExtraWidth += clamp(width_orig, -0.05f, 0.05f);

            //Also prevent being snapped to a value of the opposite direction as it looks like some weird flickering then
            overlay_width = overlay_width_orig;
        }
        else
        {
            m_DragModeSnappedExtraWidth = 0.0f;
        }
    }

    overlay_width = std::min(overlay_width, m_DragModeMaxWidth);
    vr::VROverlay()->SetOverlayWidthInMeters(m_DragModeOverlayHandle, overlay_width);

    if (m_DragModeOverlayID != k_ulOverlayID_None)
    {
        OverlayConfigData& data = OverlayManager::Get().GetConfigData(m_DragModeOverlayID);
        data.ConfigFloat[configid_float_overlay_width] = overlay_width;

        #ifndef DPLUS_UI
            //Send adjusted width to the UI app
            IPCManager::Get().PostConfigMessageToUIApp(configid_int_state_overlay_current_id_override, (int)m_DragModeOverlayID);
            IPCManager::Get().PostConfigMessageToUIApp(configid_float_overlay_width, overlay_width);
            IPCManager::Get().PostConfigMessageToUIApp(configid_int_state_overlay_current_id_override, -1);
        #endif
    }

    return overlay_width;
}

void OverlayDragger::DragSetMaxWidth(float max_width)
{
    m_DragModeMaxWidth = max_width;
}

Matrix4 OverlayDragger::DragFinish()
{
    DragUpdate();

    //Allow managed overlay origin to change after drag (used for auto-docking)
    if (m_DragModeOverlayID != k_ulOverlayID_None)
    {
        OverlayConfigData& data = OverlayManager::Get().GetConfigData(m_DragModeOverlayID);
        m_DragModeOverlayOrigin = (OverlayOrigin)data.ConfigInt[configid_int_overlay_origin];
    }

    vr::HmdMatrix34_t transform_target;
    vr::TrackingUniverseOrigin origin;

    vr::VROverlay()->GetOverlayTransformAbsolute(m_DragModeOverlayHandle, &origin, &transform_target);
    Matrix4 matrix_target_finish = transform_target;

    Matrix4 matrix_target_base = GetBaseOffsetMatrix(m_DragModeOverlayOrigin, m_DragModeOverlayOriginConfig);
    matrix_target_base.invert();

    Matrix4 matrix_target_relative = matrix_target_base * matrix_target_finish;

    //Apply to managed overlay if drag was with ID
    if (m_DragModeOverlayID != k_ulOverlayID_None)
    {
        OverlayConfigData& data = OverlayManager::Get().GetConfigData(m_DragModeOverlayID);

        //Counteract additonal offset that might've been present on the transform
        matrix_target_relative.translate_relative(-data.ConfigFloat[configid_float_overlay_offset_right],
                                                  -data.ConfigFloat[configid_float_overlay_offset_up],
                                                  -data.ConfigFloat[configid_float_overlay_offset_forward]);

        //Counteract origin offset for dashboard origin overlays
        #ifndef DPLUS_UI
        if (m_DragModeOverlayOrigin == ovrl_origin_dashboard)
        {
            if (OutputManager* outmgr = OutputManager::Get())
            {
                float height = outmgr->GetOverlayHeight(m_DragModeOverlayID);
                matrix_target_relative.translate_relative(0.0f, height / -2.0f, 0.0f);
            }
        }
        #endif

        data.ConfigTransform = matrix_target_relative;
    }

    //Reset state
    m_DragModeDeviceID          = -1;
    m_DragModeOverlayID         = k_ulOverlayID_None;
    m_DragModeOverlayHandle     = vr::k_ulOverlayHandleInvalid;
    m_DragModeSnappedExtraWidth = 0.0f;
    m_AbsoluteModeActive        = false;

    return matrix_target_relative;
}

void OverlayDragger::DragCancel()
{
    //Reset state
    m_DragModeDeviceID          = -1;
    m_DragModeOverlayID         = k_ulOverlayID_None;
    m_DragModeOverlayHandle     = vr::k_ulOverlayHandleInvalid;
    m_DragModeSnappedExtraWidth = 0.0f;
    m_AbsoluteModeActive        = false;
}

void OverlayDragger::DragGestureStart(unsigned int overlay_id)
{
    if ( (IsDragActive()) || (IsDragGestureActive()) )
        return;

    const OverlayConfigData& data = OverlayManager::Get().GetConfigData(overlay_id);

    m_DragModeDeviceID      = -1;
    m_DragModeOverlayID     = overlay_id;
    m_DragModeOverlayOrigin = (OverlayOrigin)data.ConfigInt[configid_int_overlay_origin];
    m_DragModeOverlayHandle = data.ConfigHandle[configid_handle_overlay_state_overlay_handle];

    DragGestureStartBase();
}

void OverlayDragger::DragGestureStart(vr::VROverlayHandle_t overlay_handle, OverlayOrigin overlay_origin)
{
    if ( (IsDragActive()) || (IsDragGestureActive()) )
        return;

    m_DragModeDeviceID      = -1;
    m_DragModeOverlayID     = k_ulOverlayID_None;
    m_DragModeOverlayHandle = overlay_handle;
    m_DragModeOverlayOrigin = overlay_origin;

    DragGestureStartBase();
}

void OverlayDragger::DragGestureUpdate()
{
    vr::TrackedDeviceIndex_t index_right = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
    vr::TrackedDeviceIndex_t index_left  = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);

    if ( (index_right != vr::k_unTrackedDeviceIndexInvalid) && (index_left != vr::k_unTrackedDeviceIndexInvalid) )
    {
        vr::TrackingUniverseOrigin universe_origin = vr::TrackingUniverseStanding;
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(universe_origin, vr::IVRSystemEx::GetTimeNowToPhotons(), poses, vr::k_unMaxTrackedDeviceCount);

        if ( (poses[index_right].bPoseIsValid) && (poses[index_left].bPoseIsValid) )
        {
            Matrix4 mat_right = poses[index_right].mDeviceToAbsoluteTracking;
            Matrix4 mat_left  = poses[index_left].mDeviceToAbsoluteTracking;

            //Gesture Scale
            m_DragGestureScaleDistanceLast = mat_right.getTranslation().distance(mat_left.getTranslation());

            if (m_DragGestureActive)
            {
                //Scale is just the start scale multiplied by the factor of changed controller distance
                float width = m_DragGestureScaleWidthStart * (m_DragGestureScaleDistanceLast / m_DragGestureScaleDistanceStart);
                width = std::min(width, m_DragModeMaxWidth);
                vr::VROverlay()->SetOverlayWidthInMeters(m_DragModeOverlayHandle, width);

                if (m_DragModeOverlayID != k_ulOverlayID_None)
                {
                    OverlayManager::Get().GetConfigData(m_DragModeOverlayID).ConfigFloat[configid_float_overlay_width] = width;

                    #ifndef DPLUS_UI
                    //Send adjusted width to the UI app
                    IPCManager::Get().PostConfigMessageToUIApp(configid_int_state_overlay_current_id_override, (int)m_DragModeOverlayID);
                    IPCManager::Get().PostConfigMessageToUIApp(configid_float_overlay_width, width);
                    IPCManager::Get().PostConfigMessageToUIApp(configid_int_state_overlay_current_id_override, -1);
                    #endif
                }
            }

            //Gesture Rotate
            Matrix4 matrix_rotate_current = mat_left;
            //Use up-vector multiplied by rotation matrix to avoid locking at near-up transforms
            Vector3 up = m_DragGestureRotateMatLast * Vector3(0.0f, 1.0f, 0.0f);
            up.normalize();
            //Rotation motion is taken from the differences between left controller lookat(right controller) results
            vr::IVRSystemEx::TransformLookAt(matrix_rotate_current, mat_right.getTranslation(), up);

            if (m_DragGestureActive)
            {
                //Get difference of last drag frame
                Matrix4 matrix_rotate_last_inverse = m_DragGestureRotateMatLast;
                matrix_rotate_last_inverse.setTranslation({0.0f, 0.0f, 0.0f});
                matrix_rotate_last_inverse.invert();

                Matrix4 matrix_rotate_current_at_origin = matrix_rotate_current;
                matrix_rotate_current_at_origin.setTranslation({0.0f, 0.0f, 0.0f});

                Matrix4 matrix_rotate_diff = matrix_rotate_current_at_origin * matrix_rotate_last_inverse;

                //Apply difference
                Matrix4& mat_overlay = m_DragModeMatrixTargetStart;
                Vector3 pos = mat_overlay.getTranslation();
                mat_overlay.setTranslation({0.0f, 0.0f, 0.0f});
                mat_overlay = matrix_rotate_diff * mat_overlay;
                mat_overlay.setTranslation(pos);

                vr::HmdMatrix34_t vrmat = mat_overlay.toOpenVR34();
                vr::VROverlay()->SetOverlayTransformAbsolute(m_DragModeOverlayHandle, vr::TrackingUniverseStanding, &vrmat);
            }

            m_DragGestureRotateMatLast = matrix_rotate_current;
        }
    }
}

Matrix4 OverlayDragger::DragGestureFinish()
{
    Matrix4 matrix_target_base = GetBaseOffsetMatrix(m_DragModeOverlayOrigin, m_DragModeOverlayOriginConfig);
    matrix_target_base.invert();

    Matrix4 matrix_target_relative = matrix_target_base * m_DragModeMatrixTargetStart;

    //Apply to managed overlay if drag was with ID
    if (m_DragModeOverlayID != k_ulOverlayID_None)
    {
        OverlayConfigData& data = OverlayManager::Get().GetConfigData(m_DragModeOverlayID);

        //Counteract additonal offset that might've been present on the transform
        matrix_target_relative.translate_relative(-data.ConfigFloat[configid_float_overlay_offset_right],
                                                  -data.ConfigFloat[configid_float_overlay_offset_up],
                                                  -data.ConfigFloat[configid_float_overlay_offset_forward]);

        //Counteract origin offset for dashboard origin overlays
        #ifndef DPLUS_UI
        if (m_DragModeOverlayOrigin == ovrl_origin_dashboard)
        {
            if (OutputManager* outmgr = OutputManager::Get())
            {
                float height = outmgr->GetOverlayHeight(m_DragModeOverlayID);
                matrix_target_relative.translate_relative(0.0f, height / -2.0f, 0.0f);
            }
        }
        #endif

        data.ConfigTransform = matrix_target_relative;
    }

    //Reset state
    m_DragGestureActive     = false;
    m_DragModeOverlayID     = k_ulOverlayID_None;
    m_DragModeOverlayHandle = vr::k_ulOverlayHandleInvalid;

    return matrix_target_relative;
}

void OverlayDragger::AbsoluteModeSet(bool is_active, float offset_forward)
{
    m_AbsoluteModeActive = is_active;
    m_AbsoluteModeOffsetForward = offset_forward;
}

void OverlayDragger::UpdateDashboardHMD_Y()
{
    vr::VROverlayHandle_t ovrl_handle_dplus;
    vr::VROverlay()->FindOverlay("elvissteinjr.DesktopPlusDashboard", &ovrl_handle_dplus);

    //Use dashboard dummy if available and visible. It provides a way more reliable reference point
    if ( (ovrl_handle_dplus != vr::k_ulOverlayHandleInvalid) && (vr::VROverlay()->IsOverlayVisible(ovrl_handle_dplus)) )
    {
        //Adjust offset if GamepadUI (SteamVR 2 dashboard) exists
        vr::VROverlayHandle_t handle_gamepad_ui = vr::k_ulOverlayHandleInvalid;
        vr::VROverlay()->FindOverlay("valve.steam.gamepadui.bar", &handle_gamepad_ui);

        vr::HmdMatrix34_t matrix_dplus_tab;
        vr::TrackingUniverseOrigin origin = vr::TrackingUniverseStanding;
        vr::VROverlay()->GetTransformForOverlayCoordinates(ovrl_handle_dplus, origin, {0.5f, 0.0f}, &matrix_dplus_tab);

        //Rough height difference between dashboard dummy reference point and SystemUI reference point (slightly different with GamepadUI active)
        const float height_diff = (handle_gamepad_ui != vr::k_ulOverlayHandleInvalid) ? 0.505283f : 0.575283f;
        m_DashboardHMD_Y = matrix_dplus_tab.m[1][3] + height_diff;
    }
    else //Otherwise use current headset pose. This works decently when looking straight, but drifts sligthly when not
    {
        vr::TrackingUniverseOrigin universe_origin = vr::TrackingUniverseStanding;
        vr::TrackedDevicePose_t poses[vr::k_unTrackedDeviceIndex_Hmd + 1];
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(universe_origin, 0 /*don't predict anything here*/, poses, vr::k_unTrackedDeviceIndex_Hmd + 1);

        if (poses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
        {
            Matrix4 mat_pose = poses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;

            //Offset pose 0.10 m forward to the actual center of the HMD pose. This is still pretty hacky, but minimizes deviation from not looking straight
            mat_pose.translate_relative(0.0f, 0.0f, 0.10f);

            m_DashboardHMD_Y = mat_pose.getTranslation().y;
        }
    }
}

void OverlayDragger::UpdateTempStandingPosition()
{
    vr::TrackedDevicePose_t poses[vr::k_unTrackedDeviceIndex_Hmd + 1];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, vr::IVRSystemEx::GetTimeNowToPhotons(), poses, vr::k_unTrackedDeviceIndex_Hmd + 1);

    if (poses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
    {
        Matrix4 mat_pose = poses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
        Vector3 pos_hmd = mat_pose.getTranslation();

        //Allow for slight tolerance in position changes from head movement for a more fixed reference point
        if (pos_hmd.distance(m_TempStandingPosition) > 0.05f)
        {
            m_TempStandingPosition = pos_hmd;
        }
    }
}

bool OverlayDragger::IsDragActive() const
{
    return (m_DragModeDeviceID != -1);
}

bool OverlayDragger::IsDragGestureActive() const
{
    return m_DragGestureActive;
}

int OverlayDragger::GetDragDeviceID() const
{
    return m_DragModeDeviceID;
}

unsigned int OverlayDragger::GetDragOverlayID() const
{
    return m_DragModeOverlayID;
}

vr::VROverlayHandle_t OverlayDragger::GetDragOverlayHandle() const
{
    return m_DragModeOverlayHandle;
}

const Matrix4& OverlayDragger::GetDragOverlayMatrix() const
{
    return m_DragModeMatrixTargetCurrent;
}
