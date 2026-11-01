"""Robot designs: collecting a user's URDF with its meshes, versions, and the checks that
decide whether a design can run in Gazebo."""

import shutil
from pathlib import Path

import pytest

from argus.hub.designs import ANSWERS_NAME, GAZEBO_NAME, URDF_NAME, Bundle, DesignStore, collect
from argus.setup.importer import ANSWERS_HINT

STL = """solid part
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 0.1 0 0
  vertex 0 0.1 0.05
 endloop
endfacet
endsolid part
"""

INERTIA = '<inertial><mass value="{m}"/><inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/></inertial>'


def _urdf(wheel_inertial=True, sensor_mesh="meshes/part.stl", visual_mesh="package://my_robot/meshes/shell.stl",
          base_inertia=INERTIA.format(m=5, i=0.1)):
    wheel = INERTIA.format(m=0.5, i=0.001) if wheel_inertial else ""
    return f"""<?xml version="1.0"?>
<robot name="Test_Rover">
  <link name="base_link">
    {base_inertia}
    <visual><geometry><mesh filename="{visual_mesh}"/></geometry></visual>
    <collision><origin xyz="0 0 0.1"/><geometry><box size="0.4 0.3 0.1"/></geometry></collision>
  </link>
  <link name="wheel">
    {wheel}
    <collision><geometry><cylinder radius="0.05" length="0.03"/></geometry></collision>
  </link>
  <joint name="wheel_joint" type="continuous"><parent link="base_link"/><child link="wheel"/>
    <origin xyz="0 0.17 0.05" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/></joint>
  <link name="sensor">
    <collision><geometry><mesh filename="{sensor_mesh}"/></geometry></collision>
  </link>
  <joint name="sensor_joint" type="fixed"><parent link="base_link"/><child link="sensor"/>
    <origin xyz="0.15 0 0.2"/></joint>
</robot>
"""


def _design_dir(tmp_path, **urdf_args):
    d = tmp_path / "src"
    (d / "meshes").mkdir(parents=True)
    (d / "meshes" / "part.stl").write_text(STL)
    pkg = tmp_path / "my_robot"
    (pkg / "meshes").mkdir(parents=True)
    (pkg / "meshes" / "shell.stl").write_text(STL)
    (d / "robot.urdf").write_text(_urdf(**urdf_args))
    (d / "answers.yaml").write_text(ANSWERS_HINT)
    return d, {"my_robot": pkg}


def _kinds(version):
    return [(f.severity, f.where) for f in version.findings]


def test_a_design_carries_its_meshes_and_its_version_follows_its_contents(tmp_path):
    d, packages = _design_dir(tmp_path)
    b = collect(d / "robot.urdf", d / "answers.yaml", packages=packages)
    assert b.name == "test-rover"
    assert set(b.files) == {URDF_NAME, ANSWERS_NAME, "packages/my_robot/meshes/shell.stl", "meshes/0_part.stl"}
    urdf = b.files[URDF_NAME].decode()
    assert 'filename="package://my_robot/meshes/shell.stl"' in urdf  # package URIs stay
    assert 'filename="meshes/0_part.stl"' in urdf  # file paths point inside the design
    assert b.findings == []
    same = collect(d / "robot.urdf", d / "answers.yaml", packages=packages)
    assert same.version() == b.version()
    (d / "meshes" / "part.stl").write_text(STL.replace("0.05", "0.06"))
    assert collect(d / "robot.urdf", d / "answers.yaml", packages=packages).version() != b.version()


def test_a_stored_design_is_checked_once_and_can_run(tmp_path):
    d, packages = _design_dir(tmp_path)
    store = DesignStore(tmp_path / "designs")
    v, new = store.add(collect(d / "robot.urdf", d / "answers.yaml", packages=packages))
    assert new and v.runnable, [str(f) for f in v.findings]
    assert v.profile_id == "my-robot"  # from the answers
    assert (v.path / "profile.yaml").is_file() and (v.path / "meshes" / "0_part.stl").is_file()
    assert all(f.severity == "note" for f in v.findings)
    again, new = store.add(collect(d / "robot.urdf", d / "answers.yaml", packages=packages))
    assert not new and again.version == v.version
    assert store.names() == ["test-rover"]
    assert store.get("test-rover").version == v.version
    assert store.get("test-rover", v.version[:6]).version == v.version
    assert store.profile(v)["id"] == "my-robot"


def test_a_massless_moving_link_is_an_error_and_a_massless_fixed_one_a_note(tmp_path):
    d, packages = _design_dir(tmp_path, wheel_inertial=False)
    v, _ = DesignStore(tmp_path / "designs").add(collect(d / "robot.urdf", d / "answers.yaml", packages=packages))
    assert ("error", "link wheel") in _kinds(v) and not v.runnable
    assert ("note", "link sensor") in _kinds(v)


@pytest.mark.parametrize("inertia, words", [
    (INERTIA.format(m=0, i=0.1), "positive mass"),
    ('<inertial><mass value="5"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="-0.1" iyz="0" izz="0.1"/></inertial>',
     "not positive definite"),
    ('<inertial><mass value="5"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="1"/></inertial>',
     "triangle inequality"),
], ids=["no_mass", "negative_moment", "impossible_moments"])
def test_impossible_physics_is_refused_not_patched(tmp_path, inertia, words):
    d, packages = _design_dir(tmp_path, base_inertia=inertia)
    v, _ = DesignStore(tmp_path / "designs").add(collect(d / "robot.urdf", d / "answers.yaml", packages=packages))
    assert not v.runnable
    assert any(f.severity == "error" and f.where == "link base_link" and words in f.text for f in v.findings)
    assert "ixx=\"0.1\"" in (v.path / URDF_NAME).read_text() or "ixx=\"0.01\"" in (v.path / URDF_NAME).read_text()


def test_a_missing_collision_mesh_is_an_error_and_a_missing_visual_mesh_a_warning(tmp_path):
    d, packages = _design_dir(tmp_path, visual_mesh="package://my_robot/meshes/gone.stl")
    b = collect(d / "robot.urdf", d / "answers.yaml", packages=packages)
    assert [(f.severity, "has no file meshes/gone.stl" in f.text) for f in b.findings] == [("warning", True)]
    d2, packages2 = _design_dir(tmp_path / "two", sensor_mesh="meshes/gone.stl")
    b2 = collect(d2 / "robot.urdf", d2 / "answers.yaml", packages=packages2)
    assert [f.severity for f in b2.findings] == ["error"]


def test_a_design_without_answers_cannot_run_and_says_how_to_start_one(tmp_path):
    d, packages = _design_dir(tmp_path)
    v, _ = DesignStore(tmp_path / "designs").add(collect(d / "robot.urdf", packages=packages))
    assert not v.runnable and v.profile_id == ""
    assert any(f.where == ANSWERS_NAME and "setup template" in f.text for f in v.findings)


def test_the_store_refuses_bad_names_and_files_outside_the_design(tmp_path):
    store = DesignStore(tmp_path / "designs")
    with pytest.raises(ValueError, match="design name"):
        store.add(Bundle(name="Bad Name", files={URDF_NAME: b"<robot/>"}))
    with pytest.raises(ValueError, match="outside the design"):
        store.add(Bundle(name="ok", files={"../escape.txt": b"x"}))


@pytest.mark.skipif(not Path("/opt/ros/jazzy/share/nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf").exists()
                    or shutil.which("xacro") is None,
                    reason="needs Nav2's TurtleBot3 simulation package and xacro (ROS 2 sourced)")
def test_the_installed_turtlebot3_with_its_own_gazebo_model_can_run(tmp_path):
    share = Path("/opt/ros/jazzy/share/nav2_minimal_tb3_sim")
    (tmp_path / "answers.yaml").write_text(ANSWERS_HINT)
    b = collect(share / "urdf" / "turtlebot3_waffle.urdf", tmp_path / "answers.yaml",
                share / "urdf" / "gz_waffle.sdf.xacro")
    assert GAZEBO_NAME in b.files
    assert any(p.startswith("packages/nav2_minimal_tb3_sim/") for p in b.files)
    v, _ = DesignStore(tmp_path / "designs").add(b)
    assert v.runnable, [str(f) for f in v.findings]
    # Nav2's URDF names visual meshes at paths that do not exist; its Gazebo model has the right ones.
    assert any(f.severity == "warning" and "has no file models/r200.dae" in f.text for f in v.findings)
    assert len([f for f in v.findings if "tire.dae" in f.text]) == 1


def test_collada_textures_come_with_their_mesh(tmp_path):
    d, packages = _design_dir(tmp_path, visual_mesh="meshes/shell.dae")
    (d / "meshes" / "shell.dae").write_text('<COLLADA><library_images><image id="t"><init_from>tex/skin.png'
                                            '</init_from></image></library_images></COLLADA>')
    (d / "meshes" / "tex").mkdir()
    (d / "meshes" / "tex" / "skin.png").write_bytes(b"\x89PNG")
    b = collect(d / "robot.urdf", d / "answers.yaml", packages=packages)
    stored = next(p for p in b.files if p.endswith("shell.dae"))
    assert str(Path(stored).parent / "tex" / "skin.png") in b.files
    shutil.rmtree(d / "meshes" / "tex")


def test_gazebo_warnings_are_not_taken_for_errors():
    from argus.hub.designs import _gz_messages

    # gz sdf -k on a URDF with a massless wheel (Harmonic): warnings run together, each with
    # "Error Code" in its text, and the file is still valid.
    raw = ("\x1b[1;33mWarning [parser_urdf.cc:2774]\x1b[0m Error Code 19: Msg: urdf2sdf: link[wheel] has no "
           "<inertial> block defined.\nError Code 19: Msg: urdf2sdf: parent joint[wheel_joint] ignored.\n"
           "\x1b[1;33mWarning [parser_urdf.cc:2777]\x1b[0m Error Code 19: Msg: urdf2sdf: link[wheel] is not "
           "modeled in sdf.\x1b[1;33mWarning [parser_urdf.cc:2774]\x1b[0m Error Code 19: Msg: urdf2sdf: "
           "link[wheel] has no <inertial> block defined.\nError Code 19: Msg: urdf2sdf: parent joint[wheel_joint] "
           "ignored.\nValid.\n")
    errors, warnings = _gz_messages(raw)
    assert errors == []
    assert warnings == ["urdf2sdf: link[wheel] has no <inertial> block defined. urdf2sdf: parent joint[wheel_joint] "
                        "ignored.", "urdf2sdf: link[wheel] is not modeled in sdf."]
    assert _gz_messages("Error [parser.cc:10] Error Code 4: Msg: Unable to parse.\n") == (["Unable to parse."], [])
