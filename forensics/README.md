# forensics —— 取证探针与报告索引

> 这些是**一次性探针 / 原始报告**，不是主链路入口。主链路见根目录 `README.md`。
> 脚本的 `ROOT` 是硬编码绝对路径（非 `__file__` 派生），所以**移到这里后照样能跑**；
> 但它们读写的**数据文件仍留在项目根**（见下表「留根」列）—— 这是刻意的。

## 留在根目录的数据文件（被 `os.path.join(ROOT, ...)` 锚定，不能搬）

- `_sw_baseline_legL.json`  ← 引用方：`_step_check_all.py`, `_sw_baseline.py`, `_sw_ml_xcheck.py`, `_xml_fix.py`

## forensics/joints

- `forensics/joints/_DOF_FORENSIC_REPORT.txt`
- `forensics/joints/_check_lr.py`
- `forensics/joints/_dof_forensic_fig.py`
- `forensics/joints/_joint_candidates.json`
- `forensics/joints/_joint_candidates.py`
- `forensics/joints/_joint_candidates.txt`
- `forensics/joints/_plane_faces.py`
- `forensics/joints/_slide_check.py`
- `forensics/joints/_slide_check.txt`
- `forensics/joints/_slide_range.py`
- `forensics/joints/_slide_range.txt`
- `forensics/joints/_slide_recheck.py`
- `forensics/joints/_slide_recheck.txt`
- `forensics/joints/_strap_axis_check.py`
- `forensics/joints/_strap_axis_check.txt`

## forensics/mass

- `forensics/mass/_legL_instance_recon.py`
- `forensics/mass/_legL_mass_table.csv`
- `forensics/mass/_legL_mass_table.py`
- `forensics/mass/_legL_xcheck_py_vs_sw.py`
- `forensics/mass/_legL_xcheck_py_vs_sw.txt`
- `forensics/mass/_make_sw_csv.py`
- `forensics/mass/_props_names.json`
- `forensics/mass/_targets.json`
- `forensics/mass/_targets2.json`
- `forensics/mass/_targets3.json`
- `forensics/mass/_targets_meta.json`
- `forensics/mass/_targets_single.json`

## forensics/solidworks

- `forensics/solidworks/_sw_api_names.py`
- `forensics/solidworks/_sw_api_names.txt`
- `forensics/solidworks/_sw_baseline.py`
- `forensics/solidworks/_sw_baseline.txt`
- `forensics/solidworks/_sw_cfg_meta.py`
- `forensics/solidworks/_sw_cfg_meta.txt`
- `forensics/solidworks/_sw_cfgs.py`
- `forensics/solidworks/_sw_cfgs.txt`
- `forensics/solidworks/_sw_cfgs2.py`
- `forensics/solidworks/_sw_cfgs2.txt`
- `forensics/solidworks/_sw_cfgs3.py`
- `forensics/solidworks/_sw_cfgs3.txt`
- `forensics/solidworks/_sw_export_step.py`
- `forensics/solidworks/_sw_export_step.txt`
- `forensics/solidworks/_sw_export_step2.py`
- `forensics/solidworks/_sw_export_step2.txt`
- `forensics/solidworks/_sw_ids.py`
- `forensics/solidworks/_sw_ids.txt`
- `forensics/solidworks/_sw_list_materials.py`
- `forensics/solidworks/_sw_mat_probe.py`
- `forensics/solidworks/_sw_mat_probe.txt`
- `forensics/solidworks/_sw_material_test.py`
- `forensics/solidworks/_sw_material_test.txt`
- `forensics/solidworks/_sw_materials.txt`
- `forensics/solidworks/_sw_mates.py`
- `forensics/solidworks/_sw_mates.txt`
- `forensics/solidworks/_sw_mates2.py`
- `forensics/solidworks/_sw_mates2.txt`
- `forensics/solidworks/_sw_ml_xcheck.py`
- `forensics/solidworks/_sw_ml_xcheck.txt`
- `forensics/solidworks/_sw_step_probe.py`

## forensics/step

- `forensics/step/_meshfix.py`
- `forensics/step/_meshfix2.py`
- `forensics/step/_meshfix_back.txt`
- `forensics/step/_step_check_all.py`
- `forensics/step/_step_check_all.txt`
- `forensics/step/_step_probe.py`
- `forensics/step/_step_probe_A.py`
- `forensics/step/_step_probe_A.txt`
- `forensics/step/_step_probe_B.py`
- `forensics/step/_step_probe_B.txt`
- `forensics/step/_step_probe_B_out.txt`
- `forensics/step/_step_probe_C.py`
- `forensics/step/_step_probe_C.txt`
- `forensics/step/_step_probe_C_out.txt`
- `forensics/step/_step_probe_D.py`
- `forensics/step/_step_probe_D.txt`
- `forensics/step/_step_probe_D_out.txt`
- `forensics/step/_step_probe_E.py`
- `forensics/step/_step_probe_E.txt`
- `forensics/step/_step_probe_E_out.txt`
- `forensics/step/_step_probe_F.py`
- `forensics/step/_step_probe_F.txt`
- `forensics/step/_step_probe_F_out.txt`
- `forensics/step/_xml_dump.py`
- `forensics/step/_xml_dump.txt`
- `forensics/step/_xml_fix.py`
- `forensics/step/_xml_fix.txt`
- `forensics/step/_xml_group_names.py`
- `forensics/step/_xml_group_names.txt`
- `forensics/step/_xml_inspect.py`
- `forensics/step/_xml_inspect.txt`
- `forensics/step/_xml_joints.py`
- `forensics/step/_xml_joints.txt`
- `forensics/step/_xml_vs_py.py`
- `forensics/step/_xml_vs_py.txt`

## forensics/tools

- `forensics/tools/_create.txt`
- `forensics/tools/_dirs.txt`
- `forensics/tools/_gh_upload.py`
- `forensics/tools/_survey_out.txt`
- `forensics/tools/_survey_out2.txt`
- `forensics/tools/_survey_out3.txt`
- `forensics/tools/_tidy_apply.txt`
- `forensics/tools/_tidy_dirs.py`
- `forensics/tools/_tidy_root.py`
- `forensics/tools/_trash_scratch.py`
- `forensics/tools/_tree.txt`
- `forensics/tools/_upload_survey.py`

## papers

- `papers/1 Online Adaptation Framework Enables Personalization of Exoskeleton Assistance During Locomotion in Patients Affected by Stroke.pdf`
- `papers/2609.15352v1.pdf`

