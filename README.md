# findingPath_python
## Robot tìm đường đi trong mê cung - giao diện đồ họa

## 1. Yêu cầu chi tiết 
* Cho một bản đồ phẳng xOy (góc phần tư I), trên đó người ta đặt một điểm bắt đầu
S(xs, ys) và một điểm đích đến G(xG,yG). Đồng thời đặt các chướng ngại vật là các hình
đa giác lồi sao cho các đa giác không được đặt chồng lên nhau hay có điểm chung.
Không gian bản đồ được giới hạn trong một khung hình chữ nhật có góc trái dưới trùng
09/2019

* với gốc tọa độ, độ dày của khung là 1 đơn vị. Không có điểm nào trong bản đồ được vượt
hay đè lên khung này.
Chọn và cài đặt các thuật toán để tìm kiếm đường đi ngắn nhất từ S đến G sao cho
đường đi không được cắt xuyên qua các đa giác. Đường đi có thể men theo cạnh của đa
giác nhưng không được đè lên cạnh của nó. Biểu diễn đồ họa có thể ở mức đơn giản nhất
để người sử dụng thấy được các đa giác và đường đi.


**Mức độ thực hiện được chia theo các mức như sau:**
* - Mức 1 (40%): cài đặt thành công 1 thuật toán để tìm đường đi từ S tới G. Báo
cáo lại thuật toán và quá trình chạy thử. Lưu ý, chạy thử trường hợp không có
đường đi.
* - Mức 2 (30%): cài đặt ít nhất 3 thuật toán khác nhau (ví dụ tìm kiếm mù, tham
lam, heuristic, …). Báo cáo nhận xét sự khác nhau khi chạy thử 3 thuật toán.
* - Mức 3 (30%): trên bản đồ sẽ xuất hiện thêm một số điểm khác được gọi là
điểm đón. Xuất phát từ S, sau đó đi đón tất cả các điểm này rồi đến trạng thái
G. Thứ tự các điểm đón không quan trọng. Mục tiêu là tìm ra cách để tổng
đường đi là nhỏ nhất. Báo cáo thuật toán đã áp dụng và quá trình chạy thử.
* - Mức 4 (điểm cộng 10%): các hình đa giác có thể di động được với tốc độ h
tọa độ/s. Cách thức di động có thể ở mức đơn giản nhất là tới lui một khoảng
nhỏ để đảm bảo không đè lên đa giác khác. Chạy ít nhất 1 thuật toán trên đó.
Quay video và đính kèm trực tiếp/link vào báo cáo.
* - Mức 5 (điểm cộng 10%): thể hiện mô hình trên không gian 3 chiều (3D).

## 2.Input đầu vào 
**Dữ liệu đầu vào:**
* - Dòng đầu là giới hạn của không gian, được mô tả lần lượt bởi kích thước
ngang, kích thước dọc và kích thước cao (nếu có)
* - Dòng thức hai lần lượt là tọa độ điểm bắt đầu, tọa đọa điểm kết thúc và tập
hợp các điểm đón (nếu có).
* - Dòng thứ ba là số lượng đa giác có trong không gian
* - Các dòng tiếp theo, mỗi dòng chứa một đa giác theo quy tắc:
o Đa giác là tập hợp các điểm kế nhau theo chiều kim đồng hồ. Điểm
cuối cùng sẽ được hiểu ngầm là sẽ được nối đến điểm đầu tiên để tạo
thành một đa giác lồi hợp lệ.
Mỗi số trong dữ liệu input cách nhau bởi dấu phẩy.

** example picture **
![Finding path](https://i.imgur.com/XnWkaO9.png)
