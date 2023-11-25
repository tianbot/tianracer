import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog
from zipfile import ZipFile
import pysftp
import os
import time

class FileZipperApp:
    def __init__(self, root):
        self.root = root
        self.root.title("F1tenth 线上仿真赛 作品提交")

        self.file_list = []

        # File Selection
        self.file_listbox = tk.Listbox(root, selectmode=tk.MULTIPLE, borderwidth=10, width=40)
        self.file_listbox.grid(row=0, column=0, padx=0, pady=1, sticky='w', columnspan=3)

        self.add_file_button = tk.Button(root, text="添加", command=self.add_files)
        self.add_file_button.grid(row=1, column=0, pady=5, padx=10, sticky='w')

        # Delete Button
        self.delete_button = tk.Button(root, text="删除", command=self.delete_files)
        self.delete_button.grid(row=1, column=1, pady=5, padx=10, sticky='e')

         # Configure Button
        self.configure_button = tk.Button(root, text="配置", command=self.configure_sftp)
        self.configure_button.grid(row=1, column=2, pady=5, padx=10, sticky='e')

        # Team NUMBERID Entry
        self.team_id_label = tk.Label(root, text="队伍名：")
        self.team_id_label.grid(row=2, column=0, pady=5, padx=10, sticky='w')

        self.team_id_entry = tk.Entry(root)
        self.team_id_entry.grid(row=2, column=1, pady=5, padx=10, sticky='e')

        # Target IP Entry
        self.target_ip_label = tk.Label(root, text="服务器IP:")
        self.target_ip_label.grid(row=3, column=0, pady=5, padx=10, sticky='w')

        self.target_ip_entry = tk.Entry(root)
        self.target_ip_entry.grid(row=3, column=1, pady=5, padx=10, sticky='e')

        # Zip Button
        self.zip_button = tk.Button(root, text="压缩", command=self.zip_files)
        self.zip_button.grid(row=4, column=0, pady=5, padx=10, sticky='w')
        
        # Add timestamp
        self.current_time = time.strftime("%Y-%m-%d-%H%M%S", time.localtime())

        # Upload Button
        self.upload_button = tk.Button(root, text="上传", command=self.upload_via_sftp)
        self.upload_button.grid(row=4, column=1, pady=5, padx=10, sticky='e')

        self.username = "tianbot"  # default username
        self.password = "ros"  # default password

    def add_files(self):
        files = filedialog.askopenfilenames(title="Select Files or Folders to Zip", multiple=True)
        print(files)
        for file in files:
            name = file.split('/')[-1]
            self.file_listbox.insert(tk.END, name)
            self.file_list.append(file)
        self.show_msg("已添加 " + str(files[-1]) + " 成功！")
    
    def delete_files(self):
        selected_indices = self.file_listbox.curselection()
        for index in selected_indices[::-1]:  # Iterate in reverse to avoid index issues
            self.file_listbox.delete(index)
            del self.file_list[index]
        self.show_msg("已删除选定文件。")
    
    def configure_sftp(self):
        self.username = simpledialog.askstring("配置SFTP", "请输入用户名：", initialvalue=self.username)
        self.password = simpledialog.askstring("配置SFTP", "请输入密码：", show='*', initialvalue=self.password)
        self.show_msg("SFTP配置已更新。")

    def zip_files(self):
        team_id = self.team_id_entry.get()
        if not team_id:
            self.show_error_message("请输入您的队伍名。")
            return
	
        zip_filename = f"{team_id}-{self.current_time}.zip"

        with ZipFile(zip_filename, "w") as zipf:
            for file_path in self.file_list:
                if os.path.exists(file_path):
                    self._add_to_zip(zipf, file_path)
                else:
                    self.show_error_message('路径缺失，请查看文件夹是否命名为tianracer_gazebo')
                    return

        self.show_msg("已压缩 " + str(zip_filename) + " 成功！")
        print("Files zipped successfully.")

    def _add_to_zip(self, zipf, file_path):
        if os.path.isfile(file_path):
            # If it's a file, add it to the zip
            arcname = os.path.basename(file_path)
            zipf.write(file_path, arcname=arcname)
        elif os.path.isdir(file_path):
            # If it's a directory, add its contents to the zip recursively
            for root, dirs, files in os.walk(file_path):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, file_path)
                    zipf.write(file_path, arcname=arcname)
                    
    def upload_via_sftp(self):
        team_id = self.team_id_entry.get()
        if not team_id:
            self.show_error_message("请先输入你的队名。")
            return

        target_ip = self.target_ip_entry.get()
        if not target_ip:
            self.show_error_message("请输入需要上传到的服务器IP地址。")
            return

        cnopts = pysftp.CnOpts()
        cnopts.hostkeys = None  # Disable host key checking (not recommended for production)

        try:
            with pysftp.Connection(target_ip, username=self.username, password=self.password, cnopts=cnopts) as sftp:
                sftp.chdir('./Desktop/')
                sftp.put(f"{team_id}-{self.current_time}.zip", remotepath=f"{team_id}-{self.current_time}.zip")
            print("File uploaded via SFTP.")
            self.show_msg(str(team_id + '-' + self.current_time + '.zip') + '文件上传至' + target_ip + ' 成功')
        except FileNotFoundError:   
            self.show_error_message("未发现你的压缩文件。")
        except Exception as e:
            print(f"Error: {e}")
            self.show_error_message("你输入的IP不可达，请联系工作人员")

    def show_error_message(self, message):
        messagebox.showerror("Error", message)
    
    def show_msg(self, msg):
        messagebox.showinfo("MSG", msg)


if __name__ == "__main__":
    root = tk.Tk()
    app = FileZipperApp(root)
    root.mainloop()
