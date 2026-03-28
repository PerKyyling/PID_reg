import cv2
import numpy as np
import time
import serial
from typing import Optional, Tuple
import threading


class BallTracker:
    def __init__(self, camera_index=0, table_size_cm=30.0,
                 serial_port='COM8', baudrate=9600):
        """
        Инициализация трекера шарика
        """
        # Параметры ограничения координат (можно настраивать)
        self.left_limit = -10.0  # Левая граница в см
        self.right_limit = 10.0  # Правая граница в см
        self.top_limit = -10.0  # Верхняя граница в см
        self.bottom_limit = 10.0  # Нижняя граница в см

        print(f"Текущие границы области:")
        print(f"  По X: [{self.left_limit}, {self.right_limit}] см")
        print(f"  По Y: [{self.top_limit}, {self.bottom_limit}] см")
        print("\nУправление границами в реальном времени:")
        print("  Q/W - левая граница")
        print("  A/S - правая граница")
        print("  E/R - верхняя граница")
        print("  D/F - нижняя граница")
        print("  +/- - шаг изменения (0.5 см / 2.0 см)")
        print("  Space - сбросить к значениям по умолчанию (-10, 10)")
        print("  Q - выход\n")

        # Инициализация камеры
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Получаем размеры кадра
        ret, frame = self.cap.read()
        if ret:
            self.frame_height, self.frame_width = frame.shape[:2]
            print(f"Размер кадра: {self.frame_width}x{self.frame_height}")
        else:
            self.cap.release()
            raise Exception("Не удалось получить кадр с камеры")

        # Инициализация серийного порта
        """self.serial_lock = threading.Lock()
        try:
            self.ser = serial.Serial(serial_port, baudrate=baudrate,
                                     timeout=0.1, write_timeout=0.1)
            self.ser.flush()
            print(f"Серийный порт {serial_port} открыт")
        except serial.SerialException as e:
            print(f"Ошибка открытия порта {serial_port}: {e}")
            self.ser = None"""

        self.table_size_cm = table_size_cm
        self.half_table = table_size_cm / 2.0

        # Настройки для обнаружения белого шарика
        self.min_white = np.array([225, 225, 225], dtype=np.uint8)
        self.max_white = np.array([255, 255, 255], dtype=np.uint8)

        # Минимальный размер контура
        self.min_contour_area = 50

        # Счетчики для FPS
        self.frame_counter = 0
        self.fps = 0
        self.last_fps_time = time.time()

        # Флаг для остановки
        self.running = True

        # Шаг изменения границ
        self.boundary_step = 0.5  # см

        # Вычисляем начальные границы в пикселях
        self.update_pixel_boundaries()

    def update_pixel_boundaries(self):
        """Обновляет границы области в пикселях на основе текущих лимитов в см"""
        self.left_boundary_px = int((self.left_limit + self.half_table) / self.table_size_cm * self.frame_width)
        self.right_boundary_px = int((self.right_limit + self.half_table) / self.table_size_cm * self.frame_width)
        self.top_boundary_px = int((self.top_limit + self.half_table) / self.table_size_cm * self.frame_height)
        self.bottom_boundary_px = int((self.bottom_limit + self.half_table) / self.table_size_cm * self.frame_height)

        # Убеждаемся, что границы в пределах кадра
        self.left_boundary_px = max(0, min(self.frame_width, self.left_boundary_px))
        self.right_boundary_px = max(0, min(self.frame_width, self.right_boundary_px))
        self.top_boundary_px = max(0, min(self.frame_height, self.top_boundary_px))
        self.bottom_boundary_px = max(0, min(self.frame_height, self.bottom_boundary_px))

    def adjust_boundary(self, boundary: str, increase: bool):
        """
        Изменение границы области
        boundary: 'left', 'right', 'top', 'bottom'
        increase: True - увеличить, False - уменьшить
        """
        step = self.boundary_step
        old_value = None

        if boundary == 'left':
            old_value = self.left_limit
            if increase:
                self.left_limit = min(self.right_limit - 0.1, self.left_limit + step)
            else:
                self.left_limit = max(-self.half_table, self.left_limit - step)
            print(f"Левая граница: {old_value:.1f} -> {self.left_limit:.1f} см")

        elif boundary == 'right':
            old_value = self.right_limit
            if increase:
                self.right_limit = min(self.half_table, self.right_limit + step)
            else:
                self.right_limit = max(self.left_limit + 0.1, self.right_limit - step)
            print(f"Правая граница: {old_value:.1f} -> {self.right_limit:.1f} см")

        elif boundary == 'top':
            old_value = self.top_limit
            if increase:
                self.top_limit = min(self.bottom_limit - 0.1, self.top_limit + step)
            else:
                self.top_limit = max(-self.half_table, self.top_limit - step)
            print(f"Верхняя граница: {old_value:.1f} -> {self.top_limit:.1f} см")

        elif boundary == 'bottom':
            old_value = self.bottom_limit
            if increase:
                self.bottom_limit = min(self.half_table, self.bottom_limit + step)
            else:
                self.bottom_limit = max(self.top_limit + 0.1, self.bottom_limit - step)
            print(f"Нижняя граница: {old_value:.1f} -> {self.bottom_limit:.1f} см")

        self.update_pixel_boundaries()

    def reset_boundaries(self):
        """Сброс границ к значениям по умолчанию"""
        self.left_limit = -10.0
        self.right_limit = 10.0
        self.top_limit = -10.0
        self.bottom_limit = 10.0
        self.update_pixel_boundaries()
        print("\nГраницы сброшены к значениям по умолчанию: [-10, 10] см")

    def is_point_in_bounds(self, x_px: float, y_px: float) -> bool:
        """Проверяет, находится ли точка в ограниченной области"""
        return (self.left_boundary_px <= x_px <= self.right_boundary_px and
                self.top_boundary_px <= y_px <= self.bottom_boundary_px)

    def constrain_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """Ограничивает координаты в соответствии с текущими границами"""
        x_constrained = max(self.left_limit, min(self.right_limit, x))
        y_constrained = max(self.top_limit, min(self.bottom_limit, y))
        return x_constrained, y_constrained

    def detect_ball_position(self, frame: np.ndarray) -> Optional[Tuple[float, float]]:
        """
        Обнаруживает позицию белого шарика только внутри ограниченной области
        """
        if frame is None:
            return None

        # Создаем маску для белого цвета
        white_mask = cv2.inRange(frame, self.min_white, self.max_white)

        # Создаем маску ограниченной области
        roi_mask = np.zeros_like(white_mask)
        roi_mask[self.top_boundary_px:self.bottom_boundary_px,
        self.left_boundary_px:self.right_boundary_px] = 255

        # Применяем маску области интереса
        white_mask = cv2.bitwise_and(white_mask, roi_mask)

        # Морфологические операции
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        # Находим контуры
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

            # Ищем самый большой контур
        largest_contour = max(contours, key=cv2.contourArea)

        # Фильтруем по размеру
        if cv2.contourArea(largest_contour) < self.min_contour_area:
            return None

        # Находим центр масс
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None

        # Координаты центра в пикселях
        cx_px = M["m10"] / M["m00"]
        cy_px = M["m01"] / M["m00"]

        # Проверка, что центр внутри области
        if not self.is_point_in_bounds(cx_px, cy_px):
            return None

        # Преобразуем в сантиметры
        x_cm = ((cx_px / self.frame_width) * self.table_size_cm - self.half_table)
        y_cm = ((cy_px / self.frame_height) * self.table_size_cm - self.half_table)

        # Ограничиваем координаты
        x_cm, y_cm = self.constrain_coordinates(x_cm, y_cm)

        return (round(x_cm, 6), round(y_cm, 6))

    def get_frame(self) -> Optional[np.ndarray]:
        """Получает новый кадр с камеры"""
        if not self.running:
            return None

        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def visualize_detection(self, frame: np.ndarray,
                            position: Optional[Tuple[float, float]]) -> np.ndarray:
        """
        Визуализация с затемнением области вне границ
        """
        # Расчет FPS
        self.frame_counter += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.fps = self.frame_counter
            self.frame_counter = 0
            self.last_fps_time = current_time

        # Затемняем область вне границ
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (self.frame_width, self.top_boundary_px), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, self.bottom_boundary_px), (self.frame_width, self.frame_height), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, 0), (self.left_boundary_px, self.frame_height), (0, 0, 0), -1)
        cv2.rectangle(overlay, (self.right_boundary_px, 0), (self.frame_width, self.frame_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)

        if position is not None:
            x_cm, y_cm = position

            # Преобразуем в пиксели для отрисовки
            cx_px = int((x_cm + self.half_table) / self.table_size_cm * self.frame_width)
            cy_px = int((y_cm + self.half_table) / self.table_size_cm * self.frame_height)

            # Рисуем центр
            cv2.circle(frame, (cx_px, cy_px), 5, (0, 255, 0), -1)
            cv2.circle(frame, (cx_px, cy_px), 15, (0, 255, 0), 2)

            # Выводим координаты
            text = f"X: {x_cm:.2f}, Y: {y_cm:.2f}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)

        # Рисуем границы области разными цветами для наглядности
        cv2.line(frame, (self.left_boundary_px, 0), (self.left_boundary_px, self.frame_height), (255, 0, 0),
                 2)  # Синий - лево
        cv2.line(frame, (self.right_boundary_px, 0), (self.right_boundary_px, self.frame_height), (0, 255, 0),
                 2)  # Зеленый - право
        cv2.line(frame, (0, self.top_boundary_px), (self.frame_width, self.top_boundary_px), (0, 255, 255),
                 2)  # Желтый - верх
        cv2.line(frame, (0, self.bottom_boundary_px), (self.frame_width, self.bottom_boundary_px), (255, 255, 0),
                 2)  # Голубой - низ

        # Центральные линии
        center_x = int((0 + self.half_table) / self.table_size_cm * self.frame_width)
        center_y = int((0 + self.half_table) / self.table_size_cm * self.frame_height)
        cv2.line(frame, (center_x, 0), (center_x, self.frame_height), (255, 255, 255), 1)
        cv2.line(frame, (0, center_y), (self.frame_width, center_y), (255, 255, 255), 1)

        return frame

    def send_to_serial(self, position: Tuple[float, float]):
        """Отправка координат через серийный порт"""
        if self.ser and self.ser.is_open:
            try:
                with self.serial_lock:
                    x, y = position
                    data_str = f'{x:.2f},{y:.2f}\n'
                    self.ser.write(data_str.encode('utf-8'))
            except:
                pass

    def run_continuous_detection(self):
        """
        Непрерывное обнаружение и вывод координат
        """
        print("Запуск обнаружения шарика...")
        print("Нажмите 'q' для выхода")

        last_frame_time = time.time()
        frame_interval = 1.0 / 30.0

        try:
            while self.running:
                current_time = time.time()

                if current_time - last_frame_time < frame_interval:
                    time.sleep(0.001)
                    continue

                last_frame_time = current_time

                frame = self.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                position = self.detect_ball_position(frame)
                vis_frame = self.visualize_detection(frame, position)

                if position is not None:
                    # self.send_to_serial(position)
                    print(f"Позиция: X={position[0]:.2f} см, Y={position[1]:.2f} см    ", end='\r')
                else:
                    print("Объект не обнаружен в ограниченной области    ", end='\r')

                cv2.imshow('Ball Tracking', vis_frame)

                # Обработка клавиш для настройки границ
                key = cv2.waitKey(1) & 0xFF

                if key == ord('p') or key == 27:  # 'q' или ESC
                    break
                if key == ord('q'):  # Левая граница влево
                    self.adjust_boundary('left', increase=False)
                elif key == ord('w'):  # Левая граница вправо
                    self.adjust_boundary('left', increase=True)
                elif key == ord('a'):
                    self.adjust_boundary('right', increase=False)
                elif key == ord('s'):
                    self.adjust_boundary('right', increase=True)
                elif key == ord('e'):  # Верхняя граница вверх
                    self.adjust_boundary('top', increase=True)
                elif key == ord('r'):  # Верхняя граница вниз
                    self.adjust_boundary('top', increase=False)
                elif key == ord('d'):  # Нижняя граница вверх
                    self.adjust_boundary('bottom', increase=False)
                elif key == ord('f'):  # Нижняя граница вниз
                    self.adjust_boundary('bottom', increase=True)
                elif key == ord(' '):  # Пробел - сброс
                    self.reset_boundaries()

        except KeyboardInterrupt:
            print("\nОстановка по запросу пользователя")
        finally:
            self.cleanup()

    def cleanup(self):
        """Освобождение ресурсов"""
        print("\nОчистка ресурсов...")
        self.running = False

        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

        if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
            with self.serial_lock:
                self.ser.close()

        cv2.destroyAllWindows()
        print("Ресурсы освобождены")


if __name__ == "__main__":
    try:
        tracker = BallTracker()
        tracker.run_continuous_detection()
    except Exception as e:
        print(f"Ошибка при запуске: {e}")
