
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient("localhost", 23000)
sim = client.getObject("sim")

print("Connected to CoppeliaSim")
print("Scene objects:")

for handle in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
    try:
        name = sim.getObjectAlias(handle)
        object_type = sim.getObjectType(handle)
        if object_type == sim.sceneobject_joint:
            print(f"JOINT: {name} (handle={handle})")
        elif "pioneer" in name.lower() or "wheel" in name.lower():
            print(f"ROBOT/PART: {name} (handle={handle})")
    except Exception:
        pass
