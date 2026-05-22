

---

## 1. Khởi Động Nhanh (Quick Start)

### Bước 1: Mỗi lần truy cập vào workspace

```bash
bash /home/ubuntu/auto_setup.sh
```

Ý nghĩa: Thiết lập môi trường workspace và đồng bộ các đường dẫn/tài nguyên cần thiết cho team member.

### Bước 2: Vào thư mục dự án và cập nhật code mới nhất

```bash
cd /home/ubuntu/robot_project
git pull origin main
```

### Bước 3: Chuyển sang nhánh cá nhân (Ví dụ với Người 1)

```bash
git checkout nguoi1/perception
```

### Bước 4: Đồng bộ nhánh cá nhân với nhánh main

```bash
git merge main
```

### Bước 5: Chạy mô phỏng

```bash
/isaac-sim/python.sh scripts/run_task1.py
```

---

## 2. Thành Viên Đội (Team Members)

| Vai Trò | Người Phụ Trách | Nhánh (Branch) | Nhiệm Vụ Chính | Trách Nhiệm Chuyên Môn |
| --- | --- | --- | --- | --- |
| PM / Quản Lý Dự Án | Người 5 | pm/integration | Quản lý repo, kịch bản tích hợp, thiết lập quy trình. Duyệt (Approve) PR. | Đảm bảo kiến trúc dự án, README, tài liệu chuẩn chỉnh. |
| Người 1 (N1): Camera/RGB-D + Perception | Người 1 | nguoi1/perception | Trong scene hiện tại có những vật nào, thuộc loại gì, nằm ở đâu và có đủ tin cậy để gắp không? | Xử lý Camera RGB-D, nhận diện/phân loại cấu kiện, ước lượng tư thế (pose), đánh giá độ tin cậy. |
| Người 2 (N2): Transform + Planner/FSM | Người 2 | nguoi2/planner | Robot nên gắp vật nào trước, đặt vào bin nào, và đang ở trạng thái (state) nào của task? | Đóng vai trò là Điều phối kỹ thuật (Technical Coordinator). Máy trạng thái (FSM), chọn vật thể, ánh xạ vị trí thùng (bin mapping). |
| Người 2 (N2) - Hỗ trợ: Planner Support | Người 2b | nguoi2b/planner_support | Hỗ trợ Người 2 thiết kế logic FSM và xử lý các hàm biến đổi tọa độ. | Viết các hàm tiện ích (transform_utils.py), tham gia xây dựng/tối ưu Planner logic. |
| Người 3 (N3): Motion Primitive + Pick-Place | Người 3 | nguoi3/motion | Với object pose và bin pose đã có, làm sao robot gắp vật lên và đặt đúng bin mà không rơi, không va chạm? | Nội suy quỹ đạo (waypoints), gắp-đặt (Pick-place), điều khiển lực nắm (gripper), tránh va chạm. |
| Người 4 (N4): Evaluation + Debug + Ablation | Người 4 | nguoi4/evaluation | Hệ thống Task 1 có thật sự tốt lên không, lỗi chính là gì, và bản (version) nào nên giữ lại? | Thiết kế schema log, hệ thống đánh giá (eval.py), verify_in_bin, phân loại lỗi (taxonomy), chạy batch eval, phân tích dữ liệu (Pandas/CSV). Viết report, README cuối sprint. |

### Người Đánh Giá (Reviewers - Duyệt Pull Request)

| Người Đánh Giá | Vai Trò | Quyền Hạn |
| --- | --- | --- |
| Người 5 (PM) | Quản lý Dự án | Đánh giá code (Review code), duyệt (approve) và gộp (merge) PR vào nhánh main |
| Mentor | Cố vấn/Quản trị | Đánh giá kiến trúc hệ thống, phê duyệt các thay đổi mang tính cốt lõi |
| Giảng Viên | Giảng viên/Cố vấn | Đánh giá bước cuối, phê duyệt mã nguồn trước khi nộp (submission-ready code) |

> **Lưu ý quan trọng:** Mỗi PR bắt buộc được review bởi Giảng Viên và Mentor, sau đó PM chạy eval.py để đánh giá và approve trước khi merge.

---

## 3. Cấu Trúc Dự Án (Project Structure)

```
robot_project/
|-- README.md
|-- .gitignore
|
|-- src/
|   |-- task1/
|   |   |-- perception.py        # [N1]    Nhận diện + phân loại
|   |   |-- planner.py           # [N2]    Máy trạng thái FSM + ánh xạ thùng
|   |   |-- motion.py            # [N3]    Gắp-đặt + điều khiển tay gắp
|   |   |-- eval.py              # [N4]    Đánh giá kết quả (Evaluator)
|   |   |-- logger.py            # [N4]    Ghi log có cấu trúc
|   |   |-- camera_utils.py      # [N1]    Tiện ích xử lý camera
|   |   |-- transform_utils.py   # [N2]    Biến đổi tọa độ
|   |   +-- task1_runner.py      # [N5/PM] Kịch bản chạy tích hợp
|   |
|   +-- baseline_source/         # Mã nguồn gốc tham khảo (không thay đổi)
|       |-- SceneBuilder.py
|       |-- RobotArticulation.py
|       |-- DualArmIK.py
|       |-- grasp_planner.py
|       |-- coordinate_utils.py
|       |-- DataLogger.py
|       |-- config_loader.py
|       |-- main_fixed.py
|       +-- isaac_sim_robot_interface.py
|
|-- configs/
|   |-- Part_Sorting.yaml        # Task 1 (CHÍNH)
|   |-- Conveyor_Sorting.yaml    # Task 2
|   |-- Foam_Inlaying.yaml       # Task 3
|   +-- Packing_Box.yaml         # Task 4
|
|-- scripts/
|-- logs/task1/
|-- reports/
|-- lab_outputs/
|-- tests/
|-- docs/
|-- assets -> symlink
|-- lerobot -> symlink
+-- sim_ref -> symlink
```

---

## 4. Quy Trình Git và Các Nguyên Tắc (Git Workflow & Rules)

### 4.1 Quy Ước Đặt Tên Nhánh (Branch Naming Convention)

Cú pháp: `<tên_thành_viên>/<module>-<mô-tả-ngắn-gọn>`

Ví dụ:

- `nguoi1/perception-detect-workpieces`
- `nguoi2/planner-fsm-state-machine`
- `nguoi2b/planner-transform-utils`
- `nguoi3/motion-pick-place-v1`
- `nguoi4/evaluation-episode-logger`
- `pm/integration-run-script`

### 4.2 Định Dạng Thông Điệp Commit (Commit Message Format)

Cú pháp: `<loại_thay_đổi>(<phạm_vi>): <mô_tả_ngắn_gọn>`

| Loại (Type) | Trường hợp sử dụng | Ví dụ |
| --- | --- | --- |
| feat | Thêm tính năng mới | feat(perception): detect 4 workpieces |
| fix | Sửa lỗi (bug) | fix(motion): gripper close timing |
| docs | Cập nhật tài liệu | docs(readme): add pipeline diagram |
| test | Thêm mã kiểm thử | test(eval): verify_in_bin unit test |
| refactor | Cải thiện cấu trúc mã (không thay đổi tính năng) | refactor(planner): simplify FSM |
| wip | Lưu tạm tiến độ (chưa hoàn thành) | wip: saving progress before leaving |

### 4.3 Quy Trình Làm Việc Hàng Ngày (Daily Workflow)

**KHI VÀO workspace:**

```bash
bash /home/ubuntu/auto_setup.sh
cd /home/ubuntu/robot_project
git pull origin main
git checkout <nhánh-của-mình>
git merge main
```

**TRONG LÚC LÀM VIỆC:**

```bash
git add -A
git commit -m "<loại>(<phạm_vi>): <mô_tả>"
```

**TRƯỚC KHI RỜI ĐI (BẮT BUỘC):**

```bash
git add -A
git commit -m "wip: saving progress"
git push origin <nhánh-của-mình>
```

### 4.4 Quy Trình Tạo Pull Request (PR)

**Bước 1:** Hoàn thành tính năng trên nhánh cá nhân

```bash
git add -A
git commit -m "feat(perception): detect + classify 4 workpieces"
git push origin nguoi1/perception
```

**Bước 2:** Tạo PR trên GitHub

- Chọn tab "Pull requests" → "New pull request"
- Base: `main` ← Compare: `nguoi1/perception`
- Tiêu đề: `feat(perception): detect + classify 4 workpieces`
- Mô tả: Ghi rõ những thay đổi, các tệp kết quả đầu ra, và các bài kiểm thử đã chạy
- Assign reviewers: bắt buộc assign Giảng Viên, Mentor và PM

**Bước 3:** Thông báo trên kênh nhóm

```
[PR] Người 1 tạo PR #<số> — feat(perception): detect workpieces. Link: <link PR>. Xin review: @PM @Mentor
```

**Bước 4:** Xem xét và đánh giá (Review)

- Thứ tự: Giảng Viên → Mentor → PM
- PM chạy eval.py, ghi log và đánh giá kết quả tự động, sau đó approve

**Bước 5:** Chỉnh sửa theo phản hồi (nếu có)

```bash
git add -A
git commit -m "fix(perception): address PR feedback"
git push origin nguoi1/perception
```

**Bước 6:** Approve + Merge — PM thực hiện merge trên GitHub, thông báo:

```
[MERGED] PR #<số> đã merge vào main
```

**Bước 7:** Các thành viên cập nhật lại nhánh

```bash
git checkout main && git pull
git checkout <nhánh-mình> && git merge main
```

### 4.5 PR Checklist (Người tạo PR tự kiểm tra)

- [ ] Mã nguồn chạy ổn định, không bị lỗi (crash)
- [ ] Có đầy đủ tệp kết quả/chứng cứ đính kèm (log, ảnh, CSV, JSON)
- [ ] Thông điệp commit đúng định dạng quy định
- [ ] Không tự ý thay đổi file của thành viên khác
- [ ] Không hard-code đường dẫn (dùng configs/)
- [ ] Đã kiểm thử thành công trên môi trường workspace chung

---

## 5. Quy Tắc Giao Tiếp Trên Kênh Nhóm

### 5.1 Cú Pháp Tin Nhắn

| Tag | Khi Nào Sử Dụng | Cú Pháp | Ví Dụ |
| --- | --- | --- | --- |
| [PR] | Thành viên tạo PR và thông báo | [PR] \<Tên\> tạo PR #\<số\> — \<mô_tả\>. Xin review: @\<tên\> | [PR] Người 1 tạo PR #1 — detect workpieces. @PM review |
| [MERGED] | PM thông báo PR đã được gộp | [MERGED] PR #\<số\> đã merge vào main | [MERGED] PR #1 đã merge. Mọi người pull main |
| [LỖI] | Thành viên gặp lỗi cần hỗ trợ | [LỖI] \<Tên\> gặp lỗi \<mô_tả\>, cần help | [LỖI] Người 3 gặp lỗi Sim crash khi pick-place |
| [HỎI] | Thành viên thắc mắc chung | [HỎI] \<Câu_hỏi\> | [HỎI] Camera nào dùng cho Task 1? |
| [BLOCK] | Thành viên bị tắc nghẽn | [BLOCK] \<Tên\> bị chặn bởi \<việc\>, cần @\<tên\> | [BLOCK] Người 2 cần perception output từ @Người 1 |

### 5.2 Quy Tắc Sử Dụng Workspace

- Trước khi VÀO: Luôn hỏi trên kênh nhóm xem có ai đang sử dụng không
- KHÔNG dừng (stop) workspace khi chưa hỏi ý kiến cả đội
- Nếu gặp lỗi: Chụp màn hình log lỗi và thông báo ngay. KHÔNG tự ý xóa file hệ thống hoặc file của người khác

---

## 6. Thông Số Kỹ Thuật Task 1 — Desktop Sorting

### 6.1 Mục Tiêu

Robot thực hiện gắp 4 cấu kiện (2 loại A + 2 loại B) và đặt vào đúng vị trí thùng (bin) tương ứng.

### 6.2 Cấu Hình Robot

- Tổng cộng 41 khớp (joints) | 16 bậc tự do (DOF) đang kích hoạt (2 khớp hông + 2x7 khớp tay)
- Điểm tác động cuối (End-effector): L_sixforce_link / R_sixforce_link
- Tay gắp (Gripper): Mở = -0.0215, Đóng = 0.01, Torque = 100
- Động học ngược (IK): Thư viện Pinocchio [x,y,z,roll,pitch,yaw] so với hệ tọa độ gốc robot
- Tọa độ: Dùng hàm world_to_robot() thông qua điểm neo torso_link

### 6.3 Bố Trí Môi Trường (Scene Layout)

| Đối Tượng | Vị Trí (Position) | Ghi Chú |
| --- | --- | --- |
| Robot | [0.70, -0.20, 0.90] | Hướng về phía +Y (xoay 90 độ) |
| Bàn (Table) | [0.75, 0.30, 0.50] | Chiều cao mặt bàn z ≈ 1.0m |
| Hộp/Thùng (Bins) | [1.20, 0.30, 1.05] | Vị trí cố định, không thay đổi |
| Khu Vực Cấu Kiện | Tâm [0.75, 0.28, 1.04] | Phạm vi rải: x∈[0.50,0.80], y∈[0.10,0.30] |

### 6.4 Phân Loại Cấu Kiện

- Cấu kiện A (Part A): Màu đồng/vàng nguyên bản kết hợp màu đỏ
- Cấu kiện B (Part B): Màu xanh dương kết hợp màu nguyên bản

### 6.5 Hệ Thống Camera

| Tên Camera | Vị Trí Gắn | Chức Năng |
| --- | --- | --- |
| head_stereo_L/R | head_pitch_link | Ứng viên cho xử lý RGB-D |
| head_fisheye_L/R | head_pitch_link | Camera góc rộng (Wide FOV) |
| waist_front_cam | Gốc (root) | Camera trước cố định |
| back_rear_cam | waist_pitch_link | Không sử dụng trong Task 1 |

---

## 7. Luồng Xử Lý Chính (Pipeline)

```
observe()
  -> perception.detect_parts(rgb, depth)
     -> [{object_id, class_id, confidence, pose_base, grasp_hint}]
  -> planner.select_next_object(objects, handled_ids)
  -> planner.map_class_to_bin(class_id)
  -> motion.pick_place(object_pose, bin_pose, grasp_hint)
     -> {success, duration_s, retry_count, failure_reason}
  -> evaluator.verify_in_bin(object_id, class_id)
  -> logger.log_step(...)
  -> Lặp lại cho đến khi xử lý xong 4 cấu kiện hoặc hết thời gian (timeout)
```

---

## 8. Tiêu Chí Đánh Giá (Metrics)

| Tiêu Chí | MVP | Xuất Sắc |
| --- | --- | --- |
| Thành công toàn nhiệm vụ (4/4 vào đúng thùng) | ≥ 50% | ≥ 95% |
| Tỷ lệ thả đúng thùng (correct_bin_rate) | ≥ 75% | ≥ 98% |
| Số lần thả sai thùng (wrong_bin_count) | ≤ 10 | 0 |
| Số lần làm rơi cấu kiện (drop_count) | ≤ 10 | 0 |
| Số lần va chạm (collision_count) | 0–10 | 0 |
| Số lần thử lại (retry_count) | ≤ 2 | ≤ 1 |

---

## 9. Các Đường Dẫn Quan Trọng (Key Paths)

| Mục | Đường Dẫn |
| --- | --- |
| Kho lưu trữ đội (Lưu vĩnh viễn) | /home/ubuntu/robot_project/ |
| Mã nguồn Baseline (Chỉ đọc) | /workspace/Baseline/ |
| Kịch bản khởi động Simulator | /isaac-sim/python.sh |
| Mô hình Robot (URDF) | assets/resources/robot.urdf |
| Cấu hình Task 1 | configs/Part_Sorting.yaml |
| Kịch bản kiểm tra tự động | bash /home/ubuntu/verify.sh |
| Kịch bản thiết lập ban đầu | bash /home/ubuntu/auto_setup.sh |
| Thư mục sao lưu Baseline | /home/ubuntu/backup_baseline_mods/ |

---

## 10. Tóm Tắt Kỷ Luật (Rules Summary)

- KHÔNG lập trình trực tiếp trên nhánh `main`. Phải tạo PR và chờ review mới merge.
- LUÔN commit và push lên Git TRƯỚC KHI rời workspace.
- LUÔN pull code mới nhất SAU KHI vào workspace.
- CHỈ làm việc trong thư mục `/home/ubuntu/robot_project/`.
- KHÔNG thay đổi các file trong `/workspace/` (sẽ mất khi khởi động lại).
- KHÔNG tắt workspace khi chưa có sự đồng ý của cả đội.
- Tuân thủ quy tắc thông báo trên kênh nhóm: `[PR]`, `[MERGED]`, `[LỖI]`, `[HỎI]`, `[BLOCK]`.
- Mọi PR bắt buộc có approval từ Mentor, Giảng Viên và PM trước khi merge.
- Commit message phải đúng định dạng: `type(scope): mô tả`.
- KHÔNG tự ý sửa file do thành viên khác phụ trách nếu chưa thảo luận trước.

---

## 11. Liên Kết Hữu Ích (Links)

| Tài Nguyên | Liên Kết |
| --- | --- |
| Team GitHub Repository | https://github.com/\<org\>/\<repo\> |
| Mã Nguồn Baseline | https://github.com/\<org\>/\<baseline-repo\> |
| Tài Sản Cuộc Thi (Assets) | https://huggingface.co/\<org\>/\<assets\> |
| Tài Liệu Simulator | \<link tài liệu simulator\> |
| Tài Liệu LeRobot | https://huggingface.co/docs/lerobot |

