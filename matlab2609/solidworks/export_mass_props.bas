Attribute VB_Name = "export_mass_props"
'=====================================================================================
' export_mass_props.bas
' 在 SolidWorks 装配体中批量导出零部件质量属性 -> CSV
' （列名与 matlab2609/+exo2609/params_from_csv.m 期望的格式一致）
'
' -----------------------------------------------------------------------------------
' ⚠⚠ 重要声明：本宏未在本机实际运行过（沙箱内无法自动驱动 SolidWorks GUI），
'     属于「可选加速器」。**首选路线是 SOP 里的手工/GUI 流程**，那条路一定可行。
'     如果本宏报错或数字不对，不要卡在这里，直接手工填 CSV 即可。
'
' 首次运行时请务必核对下面 3 件事：
'   1) API_UNIT_SCALE —— SolidWorks API 的 CenterOfMass/惯性张量返回单位到底是
'      m 还是文档单位(mm)，随版本/设置而异。先对**单个已知零件**（建议 电机_出轴，
'      r=22mm 的圆柱）跑一次：
'         - 若质量 ≈ 0.0969 kg 且 CoM 数值量级在 ~0.1 左右 -> 是**米**，scale = 1000
'         - 若 CoM 数值量级在 ~100 左右 -> 已经是**毫米**，把 scale 改成 1
'      惯量的 scale 同理（kg*m^2 -> kg*mm^2 是 1e6；若已是 kg*mm^2 则 1）。
'   2) swMassPropertyMoment_e 的两个枚举名是否与你的版本一致
'      （若编译报错，按提示到 SolidWorks API 帮助里核对）。
'   3) 必须先建好「髋关节参考坐标系」（原点在髋轴、Z 沿髋屈伸轴），
'      并在**该坐标系下**取 CenterOfMass 与 AboutCoordSys。否则 Izz 无意义。
'
' 用法：
'   装配体打开 -> 工具 -> 宏 -> 新建 -> 粘贴本代码 -> 运行
'   会对当前**已选中**的零部件导出；未选任何零件时提示先选。
'   每导出一批换一个分组名（leg_R / leg_L / motor_R / back ...）。
'=====================================================================================
Option Explicit

' ---- 单位换算：见上方说明，首次运行后按实测调整 ----
Private Const API_UNIT_SCALE_LEN As Double = 1000#   ' m -> mm（若已是 mm 改成 1）
Private Const API_UNIT_SCALE_INERTIA As Double = 1E+06 ' kg*m^2 -> kg*mm^2（若已是改成 1）

Sub main()
    Dim swApp As SldWorks.SldWorks
    Dim swModel As ModelDoc2
    Set swApp = Application.SldWorks
    Set swModel = swApp.ActiveDoc

    If swModel Is Nothing Then
        MsgBox "请先打开一个装配体。": Exit Sub
    End If
    If swModel.GetType <> swDocASSEMBLY Then
        MsgBox "当前文档不是装配体。": Exit Sub
    End If

    ' 取当前选择集中的零部件
    Dim swSelMgr As SelectionMgr
    Set swSelMgr = swModel.SelectionManager
    Dim nSel As Long
    nSel = swSelMgr.GetSelectedObjectCount2(-1)
    If nSel = 0 Then
        MsgBox "请先在装配体里选中要导出的零部件（可框选），再运行本宏。": Exit Sub
    End If

    Dim grp As String
    grp = Trim(InputBox("这批零件归到哪个分组？(leg_R / leg_L / motor_R / motor_L / back)", _
                        "分组名", "leg_R"))
    If grp = "" Then Exit Sub

    Dim outPath As String
    outPath = swApp.GetSaveFileName2("CSV (*.csv)|*.csv", "csv", "mass_props_" & grp)
    If outPath = "" Then Exit Sub

    Dim fnum As Integer
    fnum = FreeFile
    Open outPath For Output As #fnum
    Print #fnum, "group,part,material,mass_kg,cx_mm,cy_mm,cz_mm," & _
                 "Ixx_com,Iyy_com,Izz_com,Ixy_com,Ixz_com,Iyz_com," & _
                 "Ixx_O,Iyy_O,Izz_O,Ixy_O,Ixz_O,Iyz_O,note"

    Dim i As Long, nOK As Long
    nOK = 0
    For i = 1 To nSel
        Dim t As Long, swComp As Component2
        t = swSelMgr.GetSelectedObjectType3(i, -1)
        If t = swSelComponent Then
            Set swComp = swSelMgr.GetSelectedObject6(i, -1)
            If ExportOne(swModel, swComp, grp, fnum) Then nOK = nOK + 1
        End If
    Next i

    Close #fnum
    MsgBox "已导出 " & nOK & " 个零部件 -> " & outPath & vbCrLf & vbCrLf & _
           "请核对：质量是否与 GUI 一致？CoM 量级是否合理（21 位数据请抽一个手算）？" & vbCrLf & _
           "对每个分组各运行一次，最后把各次输出的 CSV 合并成一份。", vbInformation
End Sub

Private Function ExportOne(swModel As ModelDoc2, swComp As Component2, _
                           grp As String, fnum As Integer) As Boolean
    On Error GoTo Fail

    Dim vSel(0) As Object
    Set vSel(0) = swComp

    Dim swMP As MassProperty
    Set swMP = swModel.Extension.CreateMassProperty2(vSel)
    If swMP Is Nothing Then GoTo Fail

    ' 让质量属性在「髋关节参考坐标系」下计算（若模型里已有该坐标系，取消下一行注释并改名）
    ' Dim swCS As Feature
    ' Set swCS = swModel.FeatureByName("髋关节参考坐标系")
    ' If Not swCS Is Nothing Then swMP.SetCoordinateSystem swCS

    Dim m As Double
    m = swMP.Mass
    If m <= 0 Then
        Print #fnum, grp & "," & swComp.Name2 & ",NO_MATERIAL,0,,,,,,,,,,,,,,,," & _
                     "mass=0 -> 请在装配体里给该零件指定材质"
        Exit Function
    End If

    Dim com As Variant
    com = swMP.CenterOfMass

    Dim Ic As Variant, Io As Variant
    Ic = swMP.GetMomentOfInertia(swMassPropertyMomentAboutCenterOfMass)
    Io = swMP.GetMomentOfInertia(swMassPropertyMomentAboutCoordSys)

    Dim s As String
    s = grp & "," & swComp.Name2 & "," & swComp.GetModelDoc2.MaterialIdName & "," & _
        Fmt(m, "0.000000") & "," & _
        Fmt(com(0) * API_UNIT_SCALE_LEN, "0.0000") & "," & _
        Fmt(com(1) * API_UNIT_SCALE_LEN, "0.0000") & "," & _
        Fmt(com(2) * API_UNIT_SCALE_LEN, "0.0000") & "," & _
        Fmt(Ic(0, 0) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Ic(1, 1) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Ic(2, 2) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Ic(0, 1) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Ic(0, 2) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Ic(1, 2) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Io(0, 0) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Io(1, 1) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Io(2, 2) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Io(0, 1) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Io(0, 2) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        Fmt(Io(1, 2) * API_UNIT_SCALE_INERTIA, "0.000000") & "," & _
        "sw_api_export"
    Print #fnum, s
    ExportOne = True
    Exit Function
Fail:
    Print #fnum, grp & "," & swComp.Name2 & ",ERROR,0,,,,,,,,,,,,,,,," & _
                 "err=" & Err.Description
    ExportOne = False
End Function

Private Function Fmt(ByVal x As Variant, fmt As String) As String
    Fmt = Format$(x, fmt)
End Function
