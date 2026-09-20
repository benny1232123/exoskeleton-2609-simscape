import struct, os
base = r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\meshes"
out = []
for n in ('base.stl','leg_L.stl','leg_R.stl'):
    p = os.path.join(base,n)
    sz = os.path.getsize(p)
    with open(p,'rb') as f:
        head = f.read(84)
    txt = head[:5]
    is_ascii = txt.lower().startswith(b'solid') and b'facet' in open(p,'rb').read(2048).lower()
    tri_bin = None
    if len(head) >= 84:
        tri_bin = struct.unpack('<I', head[80:84])[0]
    out.append(f"{n}: {sz} bytes ({sz/1048576:.2f} MB)  ascii_hint={is_ascii}  tri_if_binary={tri_bin}  (84+50*tri={84+50*tri_bin if tri_bin is not None else 0})")
open(r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape\_stl_head.txt","w",encoding="utf-8").write("\n".join(out))
print("\n".join(out))
