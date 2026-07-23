import flatbuffers
import arm.ArmJointState as JS
import arm.ArmEePose as EE
import arm.PiperStatus as ST
import arm.ArmTriggerResponse as TRESP
import arm.Vec3 as Vec3mod
import arm.Quat as Quatmod
from aorta.sys import AortaHeader as AH

# schema_meta modules bundle struct creators; emulate with generated modules:
EE.CreateVec3 = Vec3mod.CreateVec3
EE.CreateQuat = Quatmod.CreateQuat

def header_off(b, node="takeover_muxer_node", seq=7, stamp=1_700_000_000_000_000_000):
    ns = b.CreateString(node)
    AH.Start(b); AH.AddPublishStampNs(b, stamp)
    AH.AddPublisherNode(b, ns); AH.AddSequence(b, seq)
    return AH.End(b)

# ---- replicate single_aorta.py fills (copy the exact bodies) ----
def fill_joint_state(name, position, velocity, effort, stamp):
    def fill(b, hoff):
        name_offs = [b.CreateString(n) for n in name]
        JS.ArmJointStateStartNameVector(b, len(name_offs))
        for o in reversed(name_offs): b.PrependUOffsetTRelative(o)
        name_vec = b.EndVector()
        JS.ArmJointStateStartPositionVector(b, len(position))
        for v in reversed(position): b.PrependFloat64(v)
        pos_vec = b.EndVector()
        JS.ArmJointStateStartVelocityVector(b, len(velocity))
        for v in reversed(velocity): b.PrependFloat64(v)
        vel_vec = b.EndVector()
        JS.ArmJointStateStartEffortVector(b, len(effort))
        for v in reversed(effort): b.PrependFloat64(v)
        eff_vec = b.EndVector()
        JS.ArmJointStateStart(b)
        JS.ArmJointStateAddAortaHeader(b, hoff)
        JS.ArmJointStateAddStampNs(b, stamp)
        JS.ArmJointStateAddName(b, name_vec)
        JS.ArmJointStateAddPosition(b, pos_vec)
        JS.ArmJointStateAddVelocity(b, vel_vec)
        JS.ArmJointStateAddEffort(b, eff_vec)
        return JS.ArmJointStateEnd(b)
    return fill

def fill_ee_pose(px,py,pz,ox,oy,oz,ow,stamp):
    def fill(b, hoff):
        EE.ArmEePoseStart(b)
        EE.ArmEePoseAddAortaHeader(b, hoff)
        EE.ArmEePoseAddStampNs(b, stamp)
        EE.ArmEePoseAddPosition(b, EE.CreateVec3(b, px, py, pz))
        EE.ArmEePoseAddOrientation(b, EE.CreateQuat(b, ox, oy, oz, ow))
        return EE.ArmEePoseEnd(b)
    return fill

def fill_status(vals):
    def fill(b, hoff):
        ST.PiperStatusStart(b)
        ST.PiperStatusAddAortaHeader(b, hoff)
        ST.PiperStatusAddCtrlMode(b, vals['ctrl_mode'])
        ST.PiperStatusAddErrCode(b, vals['err_code'])
        ST.PiperStatusAddJoint3AngleLimit(b, vals['j3'])
        ST.PiperStatusAddCommunicationStatusJoint5(b, vals['c5'])
        return ST.PiperStatusEnd(b)
    return fill

def fill_trig_resp(status, message):
    def fill(b, hoff):
        msg = b.CreateString(message)
        TRESP.ArmTriggerResponseStart(b)
        TRESP.ArmTriggerResponseAddAortaHeader(b, hoff)
        TRESP.ArmTriggerResponseAddStatus(b, status)
        TRESP.ArmTriggerResponseAddMessage(b, msg)
        return TRESP.ArmTriggerResponseEnd(b)
    return fill

def run(fill):
    b = flatbuffers.Builder(0)
    h = header_off(b)
    b.Finish(fill(b, h))
    return b.Output()

# ---- JointState roundtrip ----
name=["joint1","joint2","joint3","joint4","joint5","joint6","gripper"]
pos=[0.1*i for i in range(7)]; vel=[0.01*i for i in range(6)]; eff=[0.5*i for i in range(7)]
buf = run(fill_joint_state(name,pos,vel,eff, 123))
m = JS.ArmJointState.GetRootAs(buf, 0)
assert m.StampNs()==123
assert [m.Name(i).decode() for i in range(m.NameLength())]==name
assert [round(m.Position(i),6) for i in range(m.PositionLength())]==[round(x,6) for x in pos]
assert m.VelocityLength()==6 and m.EffortLength()==7
hdr = m.AortaHeader(); assert hdr.Sequence()==7 and hdr.PublisherNode().decode()=="takeover_muxer_node"
print(f"JointState roundtrip OK  size={len(buf)}B  seq={hdr.Sequence()}")

# ---- EePose roundtrip ----
buf = run(fill_ee_pose(1.0,2.0,3.0, 0.0,0.0,0.0,1.0, 55))
m = EE.ArmEePose.GetRootAs(buf, 0)
p = m.Position(); o = m.Orientation()
assert (p.X(),p.Y(),p.Z())==(1.0,2.0,3.0) and (o.W()==1.0) and m.StampNs()==55
print(f"EePose roundtrip OK      size={len(buf)}B  pos=({p.X()},{p.Y()},{p.Z()})")

# ---- PiperStatus roundtrip ----
buf = run(fill_status({'ctrl_mode':1,'err_code':-42,'j3':True,'c5':True}))
m = ST.PiperStatus.GetRootAs(buf, 0)
assert m.CtrlMode()==1 and m.ErrCode()==-42 and m.Joint3AngleLimit()==True and m.CommunicationStatusJoint5()==True
print(f"PiperStatus roundtrip OK size={len(buf)}B  err_code={m.ErrCode()}")

# ---- Trigger response roundtrip ----
buf = run(fill_trig_resp(0, "Arm enabled successfully."))
m = TRESP.ArmTriggerResponse.GetRootAs(buf, 0)
assert m.Status()==0 and m.Message().decode()=="Arm enabled successfully."
print(f"TriggerResp roundtrip OK size={len(buf)}B  status={m.Status()} msg='{m.Message().decode()}'")
print("\nALL FILL/DECODE PATHS VALID")
