MODULE HSLU_SimBeam
    !************************************************
    ! Module      :     HSLU_SimBeam
    ! Programmer  :     Juri Jerg
    ! Date        :     2026.04
    ! Description :     Beam geometry visualization for RobotStudio SmartComponent.
    !                   Only active on virtual controller (NOT RobOS).
    !                   Controls SmartComponent signals: ElementID, LayerID,
    !                   Activate, SwapCutA, SwapCutB, Release.
    !
    ! Signals:
    !   go_SC_ElementID  (GO, 8 bit)  - Element index
    !   go_SC_LayerID    (GO, 2 bit)  - Layer index
    !   do_SC_Activate   (DO)         - Load raw beam, attach to gripper
    !   do_SC_SwapCutA   (DO)         - Swap to after-cut-A geometry
    !   do_SC_SwapCutB   (DO)         - Swap to finished geometry
    !   do_SC_Release    (DO)         - Detach beam (stays at place position)
    !   do_SC_Reset      (DO)         - Delete all placed beams
    !
    ! PERS variables:
    !   sim_geometry_folder (string)  - Absolute STL folder path. Pushed by
    !                                   Python via r_HSLU_SimBeamReset.St1.
    !                                   SmartComponent reads this PERS instead
    !                                   of its GeometryFolder Property when set.
    !   sim_tool_name       (string)  - Tool data name for TCP offset lookup.
    !                                   Pushed via r_HSLU_SimBeamReset.St2.
    !                                   Falls back to SC ToolName Property.
    !******************** HSLU **********************

    PERS string sim_geometry_folder := "";
    PERS string sim_tool_name := "";


    !************************************************
    ! Function    :     SimBeamActivate
    ! Description :     Load raw beam STL, attach to gripper
    !                   V1 = Layer, V2 = Element
    !******************** HSLU **********************
    !
    PROC r_HSLU_SimBeamActivate()
        VAR num nLayer;
        VAR num nElement;
        !
        nLayer := bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.V1;
        nElement := bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.V2;
        !
        IF NOT RobOS() THEN
            SetGO go_SC_ElementID, nElement;
            SetGO go_SC_LayerID, nLayer;
            WaitTime 0.05;
            PulseDO\PLength:=0.2, do_SC_Activate;
            WaitTime 0.2;
        ENDIF
        !
        ! Feedback
        IF bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev>0 THEN
            TEST bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev
            CASE 1:
                r_RRC_FDone;
            DEFAULT:
                r_RRC_FError;
            ENDTEST
            r_RRC_MovMsgToSenBufRob n_RRC_ChaNr;
        ENDIF
        RETURN;
    ERROR
        ! Placeholder for Error Code...
    ENDPROC


    !************************************************
    ! Function    :     SimSwapCutA
    ! Description :     Swap to after-cut-A geometry (one end mitered)
    !******************** HSLU **********************
    !
    PROC r_HSLU_SimSwapCutA()
        !
        IF NOT RobOS() THEN
            PulseDO\PLength:=0.2, do_SC_SwapCutA;
            WaitTime 0.2;
        ENDIF
        !
        ! Feedback
        IF bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev>0 THEN
            TEST bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev
            CASE 1:
                r_RRC_FDone;
            DEFAULT:
                r_RRC_FError;
            ENDTEST
            r_RRC_MovMsgToSenBufRob n_RRC_ChaNr;
        ENDIF
        RETURN;
    ERROR
        ! Placeholder for Error Code...
    ENDPROC


    !************************************************
    ! Function    :     SimSwapCutB
    ! Description :     Swap to finished beam geometry (both ends mitered)
    !******************** HSLU **********************
    !
    PROC r_HSLU_SimSwapCutB()
        !
        IF NOT RobOS() THEN
            PulseDO\PLength:=0.2, do_SC_SwapCutB;
            WaitTime 0.2;
        ENDIF
        !
        ! Feedback
        IF bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev>0 THEN
            TEST bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev
            CASE 1:
                r_RRC_FDone;
            DEFAULT:
                r_RRC_FError;
            ENDTEST
            r_RRC_MovMsgToSenBufRob n_RRC_ChaNr;
        ENDIF
        RETURN;
    ERROR
        ! Placeholder for Error Code...
    ENDPROC


    !************************************************
    ! Function    :     SimBeamRelease
    ! Description :     Detach beam from gripper, stays at current world position.
    !                   Beam remains visible (facade accumulates).
    !******************** HSLU **********************
    !
    PROC r_HSLU_SimBeamRelease()
        !
        IF NOT RobOS() THEN
            PulseDO\PLength:=0.2, do_SC_Release;
            WaitTime 0.2;
        ENDIF
        !
        ! Feedback
        IF bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev>0 THEN
            TEST bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev
            CASE 1:
                r_RRC_FDone;
            DEFAULT:
                r_RRC_FError;
            ENDTEST
            r_RRC_MovMsgToSenBufRob n_RRC_ChaNr;
        ENDIF
        RETURN;
    ERROR
        ! Placeholder for Error Code...
    ENDPROC


    !************************************************
    ! Function    :     SimBeamReset
    ! Description :     Delete all placed beams, clear SmartComponent state.
    !                   Call at start of production run.
    !******************** HSLU **********************
    !
    PROC r_HSLU_SimBeamReset()
        VAR string sFolder;
        VAR string sTool;
        !
        ! Pull optional folder / tool overrides from the instruction payload
        ! (.St1 and .St2). Non-empty values replace the PERS defaults so the
        ! SmartComponent can pick them up during the subsequent Reset pulse.
        sFolder := bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.St1;
        sTool   := bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.St2;
        IF sFolder<>"" THEN sim_geometry_folder := sFolder; ENDIF
        IF sTool<>""   THEN sim_tool_name       := sTool;   ENDIF
        !
        IF NOT RobOS() THEN
            PulseDO\PLength:=0.2, do_SC_Reset;
            WaitTime 0.2;
        ENDIF
        !
        ! Feedback
        IF bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev>0 THEN
            TEST bm_RRC_RecBufferRob{n_RRC_ChaNr,n_RRC_ReadPtrRecBuf}.Data.F_Lev
            CASE 1:
                r_RRC_FDone;
            DEFAULT:
                r_RRC_FError;
            ENDTEST
            r_RRC_MovMsgToSenBufRob n_RRC_ChaNr;
        ENDIF
        RETURN;
    ERROR
        ! Placeholder for Error Code...
    ENDPROC

ENDMODULE
