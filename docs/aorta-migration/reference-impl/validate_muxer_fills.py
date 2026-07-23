import flatbuffers
import arm.ControlMode as CM
import arm.TakeOverEvent as EV
from aorta.sys import AortaHeader as AH

def header_off(b, node="takeover_muxer_node", seq=3, stamp=1_700_000_000_000_000_000):
    ns = b.CreateString(node)
    AH.Start(b); AH.AddPublishStampNs(b, stamp); AH.AddPublisherNode(b, ns); AH.AddSequence(b, seq)
    return AH.End(b)

# ControlMode fill (copied from take_over_aorta._publish_current_mode)
def run_mode(value, stamp):
    b = flatbuffers.Builder(0); h = header_off(b)
    data_off = b.CreateString(value)
    CM.ControlModeStart(b)
    CM.ControlModeAddAortaHeader(b, h)
    CM.ControlModeAddStampNs(b, stamp)
    CM.ControlModeAddData(b, data_off)
    b.Finish(CM.ControlModeEnd(b))
    return b.Output()

# TakeOverEvent fill (copied from _publish_event)
def run_event(et, details, stamp):
    b = flatbuffers.Builder(0); h = header_off(b)
    et_off = b.CreateString(et); det_off = b.CreateString(details)
    EV.TakeOverEventStart(b)
    EV.TakeOverEventAddAortaHeader(b, h)
    EV.TakeOverEventAddStampNs(b, stamp)
    EV.TakeOverEventAddEventType(b, et_off)
    EV.TakeOverEventAddDetails(b, det_off)
    b.Finish(EV.TakeOverEventEnd(b))
    return b.Output()

buf = run_mode("override", 99)
m = CM.ControlMode.GetRootAs(buf, 0)
assert m.Data().decode()=="override" and m.StampNs()==99 and m.AortaHeader().Sequence()==3
print(f"ControlMode roundtrip OK    size={len(buf)}B  data='{m.Data().decode()}'")

buf = run_event("takeover_triggered", "Takeover service called.", 77)
e = EV.TakeOverEvent.GetRootAs(buf, 0)
assert e.EventType().decode()=="takeover_triggered" and e.Details().decode()=="Takeover service called." and e.StampNs()==77
print(f"TakeOverEvent roundtrip OK  size={len(buf)}B  event='{e.EventType().decode()}'")
print("\nMUXER FILL PATHS VALID")
