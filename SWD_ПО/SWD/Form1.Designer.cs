namespace SWD
{
    partial class Form1
    {
        /// <summary>
        /// Обязательная переменная конструктора.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Освободить все используемые ресурсы.
        /// </summary>
        /// <param name="disposing">истинно, если управляемый ресурс должен быть удален; иначе ложно.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Код, автоматически созданный конструктором форм Windows

        /// <summary>
        /// Требуемый метод для поддержки конструктора — не изменяйте 
        /// содержимое этого метода с помощью редактора кода.
        /// </summary>
        private void InitializeComponent()
        {
            this.lblPort = new System.Windows.Forms.Label();
            this.cbPorts = new System.Windows.Forms.ComboBox();
            this.btnRefreshPorts = new System.Windows.Forms.Button();
            this.lblFile = new System.Windows.Forms.Label();
            this.tbFile = new System.Windows.Forms.TextBox();
            this.btnBrowse = new System.Windows.Forms.Button();
            this.btnStart = new System.Windows.Forms.Button();
            this.progressBar = new System.Windows.Forms.ProgressBar();
            this.lblStatus = new System.Windows.Forms.Label();
            this.tbLog = new System.Windows.Forms.TextBox();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.btnClearLog = new System.Windows.Forms.Button();
            this.btnCountFirmware = new System.Windows.Forms.Button();
            this.btnFlashFromEeprom = new System.Windows.Forms.Button();
            this.cbFirmwareList = new System.Windows.Forms.ComboBox();
            this.buttonDeleteFromEeprom = new System.Windows.Forms.Button();
            this.btnVerifyFirmware = new System.Windows.Forms.Button();
            this.btnMemoryInfo = new System.Windows.Forms.Button();
            this.btnFlashFileSwd = new System.Windows.Forms.Button();
            this.lblFirmwareName = new System.Windows.Forms.Label();
            this.tbFirmwareName = new System.Windows.Forms.TextBox();
            this.btnFormatAt25 = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // 
            // btnFormatAt25
            // 
            this.btnFormatAt25.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnFormatAt25.Location = new System.Drawing.Point(570, 408);
            this.btnFormatAt25.Name = "btnFormatAt25";
            this.btnFormatAt25.Size = new System.Drawing.Size(110, 23);
            this.btnFormatAt25.TabIndex = 21;
            this.btnFormatAt25.Text = "Format AT25";
            this.btnFormatAt25.UseVisualStyleBackColor = true;
            this.btnFormatAt25.Click += new System.EventHandler(this.btnFormatAt25_Click);

            // tbLog
            // 
            this.tbLog.Location = new System.Drawing.Point(12, 12);
            this.tbLog.Multiline = true;
            this.tbLog.Name = "tbLog";
            this.tbLog.ReadOnly = true;
            this.tbLog.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.tbLog.Size = new System.Drawing.Size(240, 405);
            this.tbLog.TabIndex = 9;
            // 
            // btnClearLog
            // 
            this.btnClearLog.Location = new System.Drawing.Point(258, 12);
            this.btnClearLog.Name = "btnClearLog";
            this.btnClearLog.Size = new System.Drawing.Size(120, 23);
            this.btnClearLog.TabIndex = 10;
            this.btnClearLog.Text = "Clear Log";
            this.btnClearLog.UseVisualStyleBackColor = true;
            this.btnClearLog.Click += new System.EventHandler(this.btnClearLog_Click);
            // 
            // lblPort
            // 
            this.lblPort.AutoSize = true;
            this.lblPort.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.lblPort.Location = new System.Drawing.Point(258, 50);
            this.lblPort.Name = "lblPort";
            this.lblPort.Size = new System.Drawing.Size(75, 18);
            this.lblPort.TabIndex = 0;
            this.lblPort.Text = "COM-port";
            // 
            // cbPorts
            // 
            this.cbPorts.FormattingEnabled = true;
            this.cbPorts.Location = new System.Drawing.Point(258, 71);
            this.cbPorts.Name = "cbPorts";
            this.cbPorts.Size = new System.Drawing.Size(120, 21);
            this.cbPorts.TabIndex = 1;
            // 
            // btnRefreshPorts
            // 
            this.btnRefreshPorts.Location = new System.Drawing.Point(258, 98);
            this.btnRefreshPorts.Name = "btnRefreshPorts";
            this.btnRefreshPorts.Size = new System.Drawing.Size(120, 23);
            this.btnRefreshPorts.TabIndex = 2;
            this.btnRefreshPorts.Text = "Update Ports";
            this.btnRefreshPorts.UseVisualStyleBackColor = true;
            // 
            // lblFile
            // 
            this.lblFile.AutoSize = true;
            this.lblFile.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.lblFile.Location = new System.Drawing.Point(258, 140);
            this.lblFile.Name = "lblFile";
            this.lblFile.Size = new System.Drawing.Size(55, 18);
            this.lblFile.TabIndex = 3;
            this.lblFile.Text = "Bin file:";
            // 
            // tbFile
            // 
            this.tbFile.Location = new System.Drawing.Point(258, 161);
            this.tbFile.Name = "tbFile";
            this.tbFile.ReadOnly = true;
            this.tbFile.Size = new System.Drawing.Size(200, 20);
            this.tbFile.TabIndex = 4;
            // 
            // btnBrowse
            // 
            this.btnBrowse.Location = new System.Drawing.Point(464, 159);
            this.btnBrowse.Name = "btnBrowse";
            this.btnBrowse.Size = new System.Drawing.Size(100, 23);
            this.btnBrowse.TabIndex = 5;
            this.btnBrowse.Text = "Browse";
            this.btnBrowse.UseVisualStyleBackColor = true;
            // 
            // lblFirmwareName
            // 
            this.lblFirmwareName.AutoSize = true;
            this.lblFirmwareName.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.lblFirmwareName.Location = new System.Drawing.Point(258, 200);
            this.lblFirmwareName.Name = "lblFirmwareName";
            this.lblFirmwareName.Size = new System.Drawing.Size(94, 15);
            this.lblFirmwareName.TabIndex = 19;
            this.lblFirmwareName.Text = "Firmware name:";
            // 
            // tbFirmwareName
            // 
            this.tbFirmwareName.Location = new System.Drawing.Point(258, 218);
            this.tbFirmwareName.MaxLength = 31;
            this.tbFirmwareName.Name = "tbFirmwareName";
            this.tbFirmwareName.Size = new System.Drawing.Size(200, 20);
            this.tbFirmwareName.TabIndex = 20;
            // 
            // btnStart
            // 
            this.btnStart.Location = new System.Drawing.Point(258, 248);
            this.btnStart.Name = "btnStart";
            this.btnStart.Size = new System.Drawing.Size(200, 28);
            this.btnStart.TabIndex = 6;
            this.btnStart.Text = "Save .bin in AT25";
            this.btnStart.UseVisualStyleBackColor = true;
            // 
            // progressBar
            // 
            this.progressBar.Location = new System.Drawing.Point(258, 282);
            this.progressBar.Name = "progressBar";
            this.progressBar.Size = new System.Drawing.Size(306, 23);
            this.progressBar.TabIndex = 7;
            // 
            // lblStatus
            // 
            this.lblStatus.AutoSize = true;
            this.lblStatus.Location = new System.Drawing.Point(258, 312);
            this.lblStatus.Name = "lblStatus";
            this.lblStatus.Size = new System.Drawing.Size(34, 13);
            this.lblStatus.TabIndex = 8;
            this.lblStatus.Text = "Ready";
            // 
            // btnCountFirmware
            // 
            this.btnCountFirmware.Location = new System.Drawing.Point(258, 350);
            this.btnCountFirmware.Name = "btnCountFirmware";
            this.btnCountFirmware.Size = new System.Drawing.Size(140, 23);
            this.btnCountFirmware.TabIndex = 12;
            this.btnCountFirmware.Text = "Update Firmware List";
            this.btnCountFirmware.UseVisualStyleBackColor = true;
            this.btnCountFirmware.Click += new System.EventHandler(this.btnCountFirmware_Click);
            // 
            // cbFirmwareList
            // 
            this.cbFirmwareList.FormattingEnabled = true;
            this.cbFirmwareList.Location = new System.Drawing.Point(404, 351);
            this.cbFirmwareList.Name = "cbFirmwareList";
            this.cbFirmwareList.Size = new System.Drawing.Size(160, 21);
            this.cbFirmwareList.TabIndex = 14;
            // 
            // btnFlashFromEeprom
            // 
            this.btnFlashFromEeprom.Location = new System.Drawing.Point(258, 379);
            this.btnFlashFromEeprom.Name = "btnFlashFromEeprom";
            this.btnFlashFromEeprom.Size = new System.Drawing.Size(140, 23);
            this.btnFlashFromEeprom.TabIndex = 13;
            this.btnFlashFromEeprom.Text = "Flash from AT25";
            this.btnFlashFromEeprom.UseVisualStyleBackColor = true;
            this.btnFlashFromEeprom.Click += new System.EventHandler(this.btnFlashFromEeprom_Click);
            // 
            // btnFlashFileSwd
            // 
            this.btnFlashFileSwd.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnFlashFileSwd.Location = new System.Drawing.Point(460, 248);
            this.btnFlashFileSwd.Name = "btnFlashFileSwd";
            this.btnFlashFileSwd.Size = new System.Drawing.Size(200, 28);
            this.btnFlashFileSwd.TabIndex = 18;
            this.btnFlashFileSwd.Text = "Flash .bin by SWD";
            this.btnFlashFileSwd.UseVisualStyleBackColor = true;
            this.btnFlashFileSwd.Click += new System.EventHandler(this.btnFlashFileSwd_Click);
            // 
            // buttonDeleteFromEeprom
            // 
            this.buttonDeleteFromEeprom.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.buttonDeleteFromEeprom.Location = new System.Drawing.Point(404, 379);
            this.buttonDeleteFromEeprom.Name = "buttonDeleteFromEeprom";
            this.buttonDeleteFromEeprom.Size = new System.Drawing.Size(160, 23);
            this.buttonDeleteFromEeprom.TabIndex = 15;
            this.buttonDeleteFromEeprom.Text = "Delete from AT25";
            this.buttonDeleteFromEeprom.UseVisualStyleBackColor = true;
            this.buttonDeleteFromEeprom.Click += new System.EventHandler(this.buttonDeleteFromEeprom_Click);
            // 
            // btnVerifyFirmware
            // 
            this.btnVerifyFirmware.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnVerifyFirmware.Location = new System.Drawing.Point(570, 350);
            this.btnVerifyFirmware.Name = "btnVerifyFirmware";
            this.btnVerifyFirmware.Size = new System.Drawing.Size(110, 23);
            this.btnVerifyFirmware.TabIndex = 16;
            this.btnVerifyFirmware.Text = "Verify CRC";
            this.btnVerifyFirmware.UseVisualStyleBackColor = true;
            this.btnVerifyFirmware.Click += new System.EventHandler(this.btnVerifyFirmware_Click);
            // 
            // btnMemoryInfo
            // 
            this.btnMemoryInfo.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnMemoryInfo.Location = new System.Drawing.Point(570, 379);
            this.btnMemoryInfo.Name = "btnMemoryInfo";
            this.btnMemoryInfo.Size = new System.Drawing.Size(110, 23);
            this.btnMemoryInfo.TabIndex = 17;
            this.btnMemoryInfo.Text = "Memory Info";
            this.btnMemoryInfo.UseVisualStyleBackColor = true;
            this.btnMemoryInfo.Click += new System.EventHandler(this.btnMemoryInfo_Click);
            // 
            
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(694, 444);
            this.Controls.Add(this.tbFirmwareName);
            this.Controls.Add(this.lblFirmwareName);
            this.Controls.Add(this.btnFlashFileSwd);
            this.Controls.Add(this.btnMemoryInfo);
            this.Controls.Add(this.btnVerifyFirmware);
            this.Controls.Add(this.buttonDeleteFromEeprom);
            this.Controls.Add(this.cbFirmwareList);
            this.Controls.Add(this.btnFlashFromEeprom);
            this.Controls.Add(this.btnCountFirmware);
            this.Controls.Add(this.btnClearLog);
            this.Controls.Add(this.tbLog);
            this.Controls.Add(this.lblStatus);
            this.Controls.Add(this.progressBar);
            this.Controls.Add(this.btnStart);
            this.Controls.Add(this.btnBrowse);
            this.Controls.Add(this.tbFile);
            this.Controls.Add(this.lblFile);
            this.Controls.Add(this.btnRefreshPorts);
            this.Controls.Add(this.cbPorts);
            this.Controls.Add(this.lblPort);
            this.Controls.Add(this.btnFormatAt25);
            this.Name = "ПМК-STM";
            this.Text = "ПМК-STM";
            this.ResumeLayout(false);
            this.PerformLayout();
        }

        #endregion

        private System.Windows.Forms.Label lblPort;
        private System.Windows.Forms.ComboBox cbPorts;
        private System.Windows.Forms.Button btnRefreshPorts;
        private System.Windows.Forms.Label lblFile;
        private System.Windows.Forms.TextBox tbFile;
        private System.Windows.Forms.Button btnBrowse;
        private System.Windows.Forms.Button btnStart;
        private System.Windows.Forms.ProgressBar progressBar;
        private System.Windows.Forms.Label lblStatus;
        private System.Windows.Forms.TextBox tbLog;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.Button btnClearLog;
        private System.Windows.Forms.Button btnCountFirmware;
        private System.Windows.Forms.Button btnFlashFromEeprom;
        private System.Windows.Forms.ComboBox cbFirmwareList;
        private System.Windows.Forms.Button buttonDeleteFromEeprom;
        private System.Windows.Forms.Button btnVerifyFirmware;
        private System.Windows.Forms.Button btnMemoryInfo;
        private System.Windows.Forms.Button btnFlashFileSwd;
        private System.Windows.Forms.Label lblFirmwareName;
        private System.Windows.Forms.TextBox tbFirmwareName;
        private System.Windows.Forms.Button btnFormatAt25;
    }
}