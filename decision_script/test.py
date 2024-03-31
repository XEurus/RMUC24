import cv2

# 打开摄像头设备
cap = cv2.VideoCapture(2)
start_time = cv2.getTickCount()
frame_count = 0
while True:
    # 读取摄像头帧
    ret, frame = cap.read()
    fps=cap.get(cv2.CAP_PROP_FPS)
    # 更新帧率计数器
    # 更新帧数
    #frame_count += 1

    # 计算帧率
    #elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
    #fps = frame_count / elapsed_time

    # 显示帧率
    print("FPS: {}".format(fps))

    # 显示帧
    cv2.imshow('Camera', frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源并关闭窗口
cap.release()
cv2.destroyAllWindows()

