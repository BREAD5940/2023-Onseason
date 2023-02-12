#SingleInstance, force
#include vjoy_lib.ahk
; Set the vjoy id of the stick to control
vjoy_id := 1
; Init the vJoy stick
vjoy_init(vjoy_id)
; NumLock is set to always on
SetNumLockState, AlwaysOn
; HOTKEYS ===
NumLock::
    VJoy_SetBtn(1, vjoy_id, 1)
    Return
NumLock up::
    VJoy_SetBtn(0, vjoy_id, 1)
    Return
NumpadDiv::
    VJoy_SetBtn(1, vjoy_id, 2)
    Return
NumpadDiv up::
    VJoy_SetBtn(0, vjoy_id, 2)
    Return
NumpadMult::
    VJoy_SetBtn(1, vjoy_id, 3)
    Return
NumpadMult up::
    VJoy_SetBtn(0, vjoy_id, 3)
    Return
Numpad7::
    VJoy_SetBtn(1, vjoy_id, 4)
    Return
Numpad7 up::
    VJoy_SetBtn(0, vjoy_id, 4)
    Return
NumpadHome::
    VJoy_SetBtn(1, vjoy_id, 4)
    Return
NumpadHome up::
    VJoy_SetBtn(0, vjoy_id, 4)
    Return
Numpad8::
    VJoy_SetBtn(1, vjoy_id, 5)
    Return
Numpad8 up::
    VJoy_SetBtn(0, vjoy_id, 5)
    Return
NumpadUp::
    VJoy_SetBtn(1, vjoy_id, 5)
    Return
NumpadUp up::
    VJoy_SetBtn(0, vjoy_id, 5)
    Return
Numpad9::
    VJoy_SetBtn(1, vjoy_id, 6)
    Return
Numpad9 up::
    VJoy_SetBtn(0, vjoy_id, 6)
    Return
NumpadPgup::
    VJoy_SetBtn(1, vjoy_id, 6)
    Return
NumpadPgup up::
    VJoy_SetBtn(0, vjoy_id, 6)
    Return
Numpad4::
    VJoy_SetBtn(1, vjoy_id, 7)
    Return
Numpad4 up::
    VJoy_SetBtn(0, vjoy_id, 7)
    Return
NumpadLeft::
    VJoy_SetBtn(1, vjoy_id, 7)
    Return
NumpadLeft up::
    VJoy_SetBtn(0, vjoy_id, 7)
    Return
Numpad5::
    VJoy_SetBtn(1, vjoy_id, 8)
    Return
Numpad5 up::
    VJoy_SetBtn(0, vjoy_id, 8)
    Return
NumpadClear::
    VJoy_SetBtn(1, vjoy_id, 8)
    Return
NumpadClear up::
    VJoy_SetBtn(0, vjoy_id, 8)
    Return
Numpad6::
    VJoy_SetBtn(1, vjoy_id, 9)
    Return
Numpad6 up::
    VJoy_SetBtn(0, vjoy_id, 9)
    Return
NumpadRight::
    VJoy_SetBtn(1, vjoy_id, 9)
    Return
NumpadRight up::
    VJoy_SetBtn(0, vjoy_id, 9)
    Return
Numpad1::
    VJoy_SetBtn(1, vjoy_id, 10)
    Return
Numpad1 up::
    VJoy_SetBtn(0, vjoy_id, 10)
    Return
NumpadEnd::
    VJoy_SetBtn(1, vjoy_id, 10)
    Return
NumpadEnd up::
    VJoy_SetBtn(0, vjoy_id, 10)
    Return
Numpad2::
    VJoy_SetBtn(1, vjoy_id, 11)
    Return
Numpad2 up::
    VJoy_SetBtn(0, vjoy_id, 11)
    Return
NumpadDown::
    VJoy_SetBtn(1, vjoy_id, 11)
    Return
NumpadDown up::
    VJoy_SetBtn(0, vjoy_id, 11)
    Return
Numpad3::
    VJoy_SetBtn(1, vjoy_id, 12)
    Return
Numpad3 up::
    VJoy_SetBtn(0, vjoy_id, 12)
    Return
NumpadPgdn::
    VJoy_SetBtn(1, vjoy_id, 12)
    Return
NumpadPgdn up::
    VJoy_SetBtn(0, vjoy_id, 12)
    Return