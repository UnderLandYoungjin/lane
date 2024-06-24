import cv2
import numpy as np
import serial
import time

def detect_lanes(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    
    height, width = frame.shape[:2]
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, int(height * 0.6)),
        (0, int(height * 0.6))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    
    cropped_edges = cv2.bitwise_and(edges, mask)
    
    lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, 50, maxLineGap=50)
    
    line_image = np.zeros_like(frame)
    lane_lines = []
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
            lane_lines.append(line[0])
    
    combined_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    
    return combined_image, lane_lines

def calculate_lane_center(lane_lines, width, height):
    left_line_x = []
    right_line_x = []
    left_line_y = []
    right_line_y = []

    for line in lane_lines:
        x1, y1, x2, y2 = line
        slope = (y2 - y1) / (x2 - x1)
        if slope < 0:
            left_line_x.extend([x1, x2])
            left_line_y.extend([y1, y2])
        else:
            right_line_x.extend([x1, x2])
            right_line_y.extend([y1, y2])

    if left_line_x and right_line_x:
        left_poly = np.polyfit(left_line_y, left_line_x, 1)
        right_poly = np.polyfit(right_line_y, right_line_x, 1)
        
        left_x_intercept = np.polyval(left_poly, height)
        right_x_intercept = np.polyval(right_poly, height)
        
        lane_center = (left_x_intercept + right_x_intercept) / 2
    else:
        lane_center = width / 2

    return lane_center

def test_camera_feed_with_lane_detection():
    cap = cv2.VideoCapture(0)
    arduino = serial.Serial('COM4', 9600, timeout=1)
    time.sleep(2)  # 아두이노 초기화 시간 대기
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return None
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 가져올 수 없습니다.")
            break

        lane_frame, lane_lines = detect_lanes(frame)
        height, width, _ = frame.shape
        lane_center = calculate_lane_center(lane_lines, width, height)
        
        # 차선 중심 표시
        cv2.line(lane_frame, (int(lane_center), height), (int(lane_center), int(height * 0.6)), (255, 0, 0), 5)
        
        # 차량 중심 표시
        car_center = width / 2
        cv2.line(lane_frame, (int(car_center), height), (int(car_center), int(height * 0.6)), (0, 0, 255), 5)
        
        # 차선 중심과 차량 중심 간의 오차 계산
        deviation = lane_center - car_center
        print(f"Deviation: {deviation}")
        
        # 아두이노로 편차 값 전송
        if deviation < -150:
            arduino.write(b'0\n')
        elif -150 <= deviation < -20:
            arduino.write(b'7\n')
        elif -20 <= deviation <= 20:
            arduino.write(b'5\n')
        elif 20 < deviation <= 150:
            arduino.write(b'3\n')
        else:
            arduino.write(b'0\n')
        
        cv2.imshow('Lane Detection', lane_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()

test_camera_feed_with_lane_detection()
