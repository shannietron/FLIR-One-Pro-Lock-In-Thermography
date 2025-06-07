#!/usr/bin/env python3
"""
FLIR Real-time Image Viewer
Monitors RAMDisk for new thermal/real images and displays them in real-time GUI
"""

import os
import time
import threading
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import argparse

class FLIRViewer:
    def __init__(self, image_type='thermal', ramdisk_path='/mnt/RAMDisk'):
        self.image_type = image_type
        self.ramdisk_path = Path(ramdisk_path)
        self.thermal_file = self.ramdisk_path / 'thermal.jpg'
        self.real_file = self.ramdisk_path / 'real.jpg'

        # Track last modification times to detect new files
        self.last_thermal_mtime = 0
        self.last_real_mtime = 0

        # Threading control
        self.running = False
        self.monitor_thread = None

        # GUI setup
        self.setup_gui()

        # Frame counter for FPS calculation
        self.frame_count = 0
        self.start_time = time.time()

    def setup_gui(self):
        """Initialize the GUI"""
        self.root = tk.Tk()
        self.root.title(f"FLIR Viewer - {self.image_type.title()}")
        self.root.geometry("800x600")

        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)

        # Control panel
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        # Image type selection
        ttk.Label(control_frame, text="Image Type:").grid(row=0, column=0, padx=(0, 5))
        self.image_var = tk.StringVar(value=self.image_type)
        image_combo = ttk.Combobox(control_frame, textvariable=self.image_var, 
                                   values=['thermal', 'real', 'both'], state='readonly', width=10)
        image_combo.grid(row=0, column=1, padx=(0, 10))
        image_combo.bind('<<ComboboxSelected>>', self.on_image_type_change)

        # Start/Stop button
        self.start_button = ttk.Button(control_frame, text="Start", command=self.toggle_monitoring)
        self.start_button.grid(row=0, column=2, padx=(0, 10))

        # Status label
        self.status_label = ttk.Label(control_frame, text="Ready")
        self.status_label.grid(row=0, column=3, padx=(0, 10))

        # FPS label
        self.fps_label = ttk.Label(control_frame, text="FPS: 0.0")
        self.fps_label.grid(row=0, column=4)

        # Image display area
        if self.image_var.get() == 'both':
            # Two image panels side by side
            self.setup_dual_display(main_frame)
        else:
            # Single image panel
            self.setup_single_display(main_frame)

    def setup_single_display(self, parent):
        """Setup single image display"""
        self.image_frame = ttk.Frame(parent, relief='sunken', borderwidth=2)
        self.image_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.image_label = ttk.Label(self.image_frame, text="No image loaded", anchor='center')
        self.image_label.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.image_frame.columnconfigure(0, weight=1)
        self.image_frame.rowconfigure(0, weight=1)

    def setup_dual_display(self, parent):
        """Setup dual image display for both thermal and real"""
        # Thermal image
        thermal_frame = ttk.LabelFrame(parent, text="Thermal", padding="5")
        thermal_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))

        self.thermal_label = ttk.Label(thermal_frame, text="No thermal image", anchor='center')
        self.thermal_label.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        thermal_frame.columnconfigure(0, weight=1)
        thermal_frame.rowconfigure(0, weight=1)

        # Real image
        real_frame = ttk.LabelFrame(parent, text="Real", padding="5")
        real_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))

        self.real_label = ttk.Label(real_frame, text="No real image", anchor='center')
        self.real_label.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        real_frame.columnconfigure(0, weight=1)
        real_frame.rowconfigure(0, weight=1)

    def on_image_type_change(self, event=None):
        """Handle image type selection change"""
        new_type = self.image_var.get()
        if new_type != self.image_type:
            self.image_type = new_type
            self.root.title(f"FLIR Viewer - {self.image_type.title()}")

            # Recreate display based on new type
            for widget in self.root.winfo_children():
                if isinstance(widget, ttk.Frame):
                    for child in widget.winfo_children():
                        if hasattr(child, 'grid_info') and child.grid_info().get('row') == 1:
                            child.destroy()

            main_frame = self.root.winfo_children()[0]
            if new_type == 'both':
                self.setup_dual_display(main_frame)
            else:
                self.setup_single_display(main_frame)

    def toggle_monitoring(self):
        """Start or stop the file monitoring"""
        if not self.running:
            self.start_monitoring()
        else:
            self.stop_monitoring()

    def start_monitoring(self):
        """Start monitoring the RAMDisk for new images"""
        if not self.ramdisk_path.exists():
            messagebox.showerror("Error", f"RAMDisk path not found: {self.ramdisk_path}")
            return

        self.running = True
        self.start_button.config(text="Stop")
        self.status_label.config(text="Monitoring...")
        self.frame_count = 0
        self.start_time = time.time()

        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_files, daemon=True)
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop monitoring"""
        self.running = False
        self.start_button.config(text="Start")
        self.status_label.config(text="Stopped")

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)

    def monitor_files(self):
        """Monitor files for changes in a separate thread"""
        while self.running:
            try:
                # Check for new thermal image
                if self.image_type in ['thermal', 'both'] and self.thermal_file.exists():
                    thermal_mtime = self.thermal_file.stat().st_mtime
                    if thermal_mtime > self.last_thermal_mtime:
                        self.last_thermal_mtime = thermal_mtime
                        self.load_and_display_image(self.thermal_file, 'thermal')

                # Check for new real image
                if self.image_type in ['real', 'both'] and self.real_file.exists():
                    real_mtime = self.real_file.stat().st_mtime
                    if real_mtime > self.last_real_mtime:
                        self.last_real_mtime = real_mtime
                        self.load_and_display_image(self.real_file, 'real')

                # Update FPS counter
                self.update_fps()

            except Exception as e:
                print(f"Error monitoring files: {e}")

            # Small delay to prevent excessive CPU usage
            time.sleep(0.01)  # 100Hz polling rate

    def load_and_display_image(self, file_path, img_type):
        """Load and display an image"""
        try:
            # Load image
            pil_image = Image.open(file_path)

            # Scale image to fit display while maintaining aspect ratio
            display_size = (400, 300) if self.image_type == 'both' else (600, 450)
            pil_image.thumbnail(display_size, Image.Resampling.LANCZOS)

            # Convert to PhotoImage
            photo = ImageTk.PhotoImage(pil_image)

            # Update GUI in main thread
            self.root.after(0, self.update_display, photo, img_type)

            self.frame_count += 1

        except Exception as e:
            print(f"Error loading image {file_path}: {e}")

    def update_display(self, photo, img_type):
        """Update the display with new image (called in main thread)"""
        try:
            if self.image_type == 'both':
                if img_type == 'thermal':
                    self.thermal_label.config(image=photo, text="")
                    self.thermal_label.image = photo  # Keep a reference
                elif img_type == 'real':
                    self.real_label.config(image=photo, text="")
                    self.real_label.image = photo  # Keep a reference
            else:
                self.image_label.config(image=photo, text="")
                self.image_label.image = photo  # Keep a reference

        except Exception as e:
            print(f"Error updating display: {e}")

    def update_fps(self):
        """Update FPS display"""
        if self.frame_count > 0:
            elapsed = time.time() - self.start_time
            if elapsed > 0:
                fps = self.frame_count / elapsed
                self.root.after(0, lambda: self.fps_label.config(text=f"FPS: {fps:.1f}"))

    def run(self):
        """Start the GUI event loop"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        """Handle window closing"""
        self.stop_monitoring()
        self.root.destroy()

def main():
    parser = argparse.ArgumentParser(description='FLIR Real-time Image Viewer')
    parser.add_argument('--type', choices=['thermal', 'real', 'both'], 
                        default='thermal', help='Image type to display')
    parser.add_argument('--path', default='/mnt/RAMDisk', 
                        help='Path to RAMDisk directory')

    args = parser.parse_args()

    viewer = FLIRViewer(image_type=args.type, ramdisk_path=args.path)
    viewer.run()

if __name__ == '__main__':
    main()
