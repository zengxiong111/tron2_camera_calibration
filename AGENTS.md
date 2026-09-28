# Working agreements

Keep authored English Markdown and Simplified Chinese `.zh-CN.md` versions in sync. Preserve third-party notices and source attribution, including the original MIT copyright in `LICENSE`.

The tools support offline camera calibration. Live acquisition is read-only and must remain opt-in; imports and offline solves must not connect to hardware. Do not add robot motion commands or claim physical validation from a solver fit alone.

The bundled URDF/MJCF snapshots omit referenced meshes and are for kinematic calculations only. Do not describe them as collision, rendering, or physics assets. Keep local machine profiles, captures, fitted results, logs and credentials out of Git and package distributions.

Do not import the archived `dexpipe` source tree. Keep wheel metadata and packaged assets aligned with the README. For code or packaging changes, run `python -m pytest -q`, build the wheel, inspect it for required config assets and smoke-test all installed CLI entrypoints from outside the checkout.
