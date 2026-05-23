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
            this.btnSaveToEeprom = new System.Windows.Forms.Button();
            this.btnCountFirmware = new System.Windows.Forms.Button();
            this.btnFlashFromEeprom = new System.Windows.Forms.Button();
            this.cbFirmwareList = new System.Windows.Forms.ComboBox();
            this.buttonDeleteFromEeprom = new System.Windows.Forms.Button();
            this.btnVerifyFirmware = new System.Windows.Forms.Button();
            this.btnMemoryInfo = new System.Windows.Forms.Button();
            this.btnFlashFileSwd = new System.Windows.Forms.Button();
            this.lblFirmwareName = new System.Windows.Forms.Label();
            this.tbFirmwareName = new System.Windows.Forms.TextBox();

            this.SuspendLayout();
            // 
            // lblPort
            // 
            this.lblPort.AutoSize = true;
            this.lblPort.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.lblPort.Location = new System.Drawing.Point(278, 15);
            this.lblPort.Name = "lblPort";
            this.lblPort.Size = new System.Drawing.Size(75, 18);
            this.lblPort.TabIndex = 0;
            this.lblPort.Text = "COM-port";
            // 
            // cbPorts
            // 
            this.cbPorts.FormattingEnabled = true;
            this.cbPorts.Location = new System.Drawing.Point(273, 48);
            this.cbPorts.Name = "cbPorts";
            this.cbPorts.Size = new System.Drawing.Size(192, 21);
            this.cbPorts.TabIndex = 1;
            // 
            // btnRefreshPorts
            // 
            this.btnRefreshPorts.Location = new System.Drawing.Point(273, 131);
            this.btnRefreshPorts.Name = "btnRefreshPorts";
            this.btnRefreshPorts.Size = new System.Drawing.Size(192, 22);
            this.btnRefreshPorts.TabIndex = 2;
            this.btnRefreshPorts.Text = "Update";
            this.btnRefreshPorts.UseVisualStyleBackColor = true;
            // 
            // lblFile
            // 
            this.lblFile.AutoSize = true;
            this.lblFile.Font = new System.Drawing.Font("Microsoft Sans Serif", 11F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.lblFile.Location = new System.Drawing.Point(270, 169);
            this.lblFile.Name = "lblFile";
            this.lblFile.Size = new System.Drawing.Size(55, 18);
            this.lblFile.TabIndex = 3;
            this.lblFile.Text = "Bin file:";
            // 
            // tbFile
            // 
            this.tbFile.Location = new System.Drawing.Point(273, 190);
            this.tbFile.Name = "tbFile";
            this.tbFile.ReadOnly = true;
            this.tbFile.Size = new System.Drawing.Size(192, 20);
            this.tbFile.TabIndex = 4;
            // 
            // btnBrowse
            // 
            this.btnBrowse.Location = new System.Drawing.Point(273, 216);
            this.btnBrowse.Name = "btnBrowse";
            this.btnBrowse.Size = new System.Drawing.Size(192, 21);
            this.btnBrowse.TabIndex = 5;
            this.btnBrowse.Text = "Browse";
            this.btnBrowse.UseVisualStyleBackColor = true;
            // 
            // lblFirmwareName
            // 
            this.lblFirmwareName.AutoSize = true;
            this.lblFirmwareName.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(204)));
            this.lblFirmwareName.Location = new System.Drawing.Point(270, 244);
            this.lblFirmwareName.Name = "lblFirmwareName";
            this.lblFirmwareName.Size = new System.Drawing.Size(94, 15);
            this.lblFirmwareName.TabIndex = 19;
            this.lblFirmwareName.Text = "Firmware name:";
            // 
            // tbFirmwareName
            // 
            this.tbFirmwareName.Location = new System.Drawing.Point(273, 264);
            this.tbFirmwareName.MaxLength = 31;
            this.tbFirmwareName.Name = "tbFirmwareName";
            this.tbFirmwareName.Size = new System.Drawing.Size(192, 20);
            this.tbFirmwareName.TabIndex = 20;
            // 
            // 
            // btnStart
            // 
            this.btnStart.Location = new System.Drawing.Point(273, 305);
            this.btnStart.Name = "btnStart";
            this.btnStart.Size = new System.Drawing.Size(192, 28);
            this.btnStart.TabIndex = 6;
            this.btnStart.Text = "Save .bin in AT25";
            this.btnStart.UseVisualStyleBackColor = true;
            // 
            // progressBar
            // 
            this.progressBar.Location = new System.Drawing.Point(273, 339);
            this.progressBar.Name = "progressBar";
            this.progressBar.Size = new System.Drawing.Size(192, 21);
            this.progressBar.TabIndex = 7;
            // 
            // lblStatus
            // 
            this.lblStatus.AutoSize = true;
            this.lblStatus.Location = new System.Drawing.Point(278, 374);
            this.lblStatus.Name = "lblStatus";
            this.lblStatus.Size = new System.Drawing.Size(34, 13);
            this.lblStatus.TabIndex = 8;
            this.lblStatus.Text = "Finish";
            // 
            // tbLog
            // 
            this.tbLog.Location = new System.Drawing.Point(9, 15);
            this.tbLog.Multiline = true;
            this.tbLog.Name = "tbLog";
            this.tbLog.ReadOnly = true;
            this.tbLog.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.tbLog.Size = new System.Drawing.Size(241, 386);
            this.tbLog.TabIndex = 9;
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // btnClearLog
            // 
            this.btnClearLog.Location = new System.Drawing.Point(477, 46);
            this.btnClearLog.Name = "btnClearLog";
            this.btnClearLog.Size = new System.Drawing.Size(192, 22);
            this.btnClearLog.TabIndex = 10;
            this.btnClearLog.Text = "Clear";
            this.btnClearLog.UseVisualStyleBackColor = true;
            this.btnClearLog.Click += new System.EventHandler(this.btnClearLog_Click);
            // 
            // btnSaveToEeprom
           
            // btnCountFirmware
            // 
            this.btnCountFirmware.Location = new System.Drawing.Point(477, 159);
            this.btnCountFirmware.Name = "btnCountFirmware";
            this.btnCountFirmware.Size = new System.Drawing.Size(192, 22);
            this.btnCountFirmware.TabIndex = 12;
            this.btnCountFirmware.Text = "Update firmware";
            this.btnCountFirmware.UseVisualStyleBackColor = true;
            this.btnCountFirmware.Click += new System.EventHandler(this.btnCountFirmware_Click);
            // 
            // btnFlashFromEeprom
            // 
            this.btnFlashFromEeprom.Location = new System.Drawing.Point(477, 250);
            this.btnFlashFromEeprom.Name = "btnFlashFromEeprom";
            this.btnFlashFromEeprom.Size = new System.Drawing.Size(192, 22);
            this.btnFlashFromEeprom.TabIndex = 13;
            this.btnFlashFromEeprom.Text = "Flash from AT25";
            this.btnFlashFromEeprom.UseVisualStyleBackColor = true;
            this.btnFlashFromEeprom.Click += new System.EventHandler(this.btnFlashFromEeprom_Click);
            // 
            // cbFirmwareList
            // 
            this.cbFirmwareList.FormattingEnabled = true;
            this.cbFirmwareList.Location = new System.Drawing.Point(477, 200);
            this.cbFirmwareList.Name = "cbFirmwareList";
            this.cbFirmwareList.Size = new System.Drawing.Size(192, 21);
            this.cbFirmwareList.TabIndex = 14;
            // 
            // buttonDeleteFromEeprom
            // 
            this.buttonDeleteFromEeprom.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.buttonDeleteFromEeprom.Location = new System.Drawing.Point(477, 283);
            this.buttonDeleteFromEeprom.Name = "buttonDeleteFromEeprom";
            this.buttonDeleteFromEeprom.Size = new System.Drawing.Size(192, 22);
            this.buttonDeleteFromEeprom.TabIndex = 15;
            this.buttonDeleteFromEeprom.Text = "Delete from EEPROM";
            this.buttonDeleteFromEeprom.UseVisualStyleBackColor = true;
            this.buttonDeleteFromEeprom.Click += new System.EventHandler(this.buttonDeleteFromEeprom_Click);
            // 
            // btnVerifyFirmware
            // 
            this.btnVerifyFirmware.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnVerifyFirmware.Location = new System.Drawing.Point(477, 311);
            this.btnVerifyFirmware.Name = "btnVerifyFirmware";
            this.btnVerifyFirmware.Size = new System.Drawing.Size(192, 22);
            this.btnVerifyFirmware.TabIndex = 16;
            this.btnVerifyFirmware.Text = "Verify CRC";
            this.btnVerifyFirmware.UseVisualStyleBackColor = true;
            this.btnVerifyFirmware.Click += new System.EventHandler(this.btnVerifyFirmware_Click);
            // 
            // btnMemoryInfo
            // 
            this.btnMemoryInfo.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnMemoryInfo.Location = new System.Drawing.Point(477, 339);
            this.btnMemoryInfo.Name = "btnMemoryInfo";
            this.btnMemoryInfo.Size = new System.Drawing.Size(192, 22);
            this.btnMemoryInfo.TabIndex = 17;
            this.btnMemoryInfo.Text = "Memory Info";
            this.btnMemoryInfo.UseVisualStyleBackColor = true;
            this.btnMemoryInfo.Click += new System.EventHandler(this.btnMemoryInfo_Click);
            // 
            // btnFlashFileSwd
            // 
            this.btnFlashFileSwd.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnFlashFileSwd.Location = new System.Drawing.Point(477, 367);
            this.btnFlashFileSwd.Name = "btnFlashFileSwd";
            this.btnFlashFileSwd.Size = new System.Drawing.Size(192, 22);
            this.btnFlashFileSwd.TabIndex = 18;
            this.btnFlashFileSwd.Text = "Flash .bin by SWD";
            this.btnFlashFileSwd.UseVisualStyleBackColor = true;
            this.btnFlashFileSwd.Click += new System.EventHandler(this.btnFlashFileSwd_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(681, 429);
            this.Controls.Add(this.tbFirmwareName);
            this.Controls.Add(this.lblFirmwareName);
            this.Controls.Add(this.btnFlashFileSwd);
            this.Controls.Add(this.btnMemoryInfo);
            this.Controls.Add(this.btnVerifyFirmware);
            this.Controls.Add(this.buttonDeleteFromEeprom);
            this.Controls.Add(this.cbFirmwareList);
            this.Controls.Add(this.btnFlashFromEeprom);
            this.Controls.Add(this.btnCountFirmware);
            this.Controls.Add(this.btnSaveToEeprom);
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
            this.Name = "Form1";
            this.Text = "Form1";
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
        private System.Windows.Forms.Button btnSaveToEeprom;
        private System.Windows.Forms.Button btnCountFirmware;
        private System.Windows.Forms.Button btnFlashFromEeprom;
        private System.Windows.Forms.ComboBox cbFirmwareList;
        private System.Windows.Forms.Button buttonDeleteFromEeprom;
        private System.Windows.Forms.Button btnVerifyFirmware;
        private System.Windows.Forms.Button btnMemoryInfo;
        private System.Windows.Forms.Button btnFlashFileSwd;
        private System.Windows.Forms.Label lblFirmwareName;
        private System.Windows.Forms.TextBox tbFirmwareName;
    }
}

