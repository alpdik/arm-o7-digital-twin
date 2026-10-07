# Source assets

The integration code in this package is Apache-2.0 licensed. The two model sources have
different provenance:

- `meshes/arm/*` and `urdf/ARM1-460.original.urdf` came from the user-provided archive
  `ARM1.5_URDF (1).rar` (SHA-256
  `DFBD23ACB62C2565CAAA16ECBC4ABDBB0C9F53FB04660C168FBE95AE629933A6`). No asset
  license was present in that archive; redistribution rights must be confirmed by the user.
- `meshes/o7/*` and `urdf/linkerhand_o7v3_right.original.urdf` came from
  `linker-bot/linkerhand-urdf`, commit
  `075cc7d42cc1e756bdcbece0fc069a0779fc5237`, directory `O7/right`, under that
  repository's Apache-2.0 license.

The `*.macro.xacro` files are mechanical adaptations: package mesh URIs were repaired,
names were normalized, the two roots were joined by a configurable fixed transform, and
non-zero **simulation-only provisional** arm effort/velocity limits were supplied because
the SolidWorks export contained zeros. Those provisional limits must not be treated as
certified physical-hardware limits.

Collision meshes in `meshes/*_collision` are conservative convex hulls generated from
the corresponding original mesh. Visual geometry remains untouched. The hulls are
closed/watertight and reduce collision triangles from about 1.6 million to roughly
60 thousand; validate grasp-contact fidelity against the physical parts before relying
on contact simulation.
