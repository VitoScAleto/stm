using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace SWD
{
    public partial class Form1 : Form
    {
        private const int BaudRate = 115200;
        private const int ChunkSize = 4;
        private const int AckTimeoutMs = 1000;
        private const int MaxRetries = 3;

        private const byte CmdCheck = (byte)'C';
        private const byte CmdStart = (byte)'S';
        private const byte CmdEnd = (byte)'E';

        private const byte CmdEepromSave = (byte)'W';
        private const byte CmdEepromCount = (byte)'N';
        private const byte CmdEepromRead = (byte)'R';

        private const byte RespOk = (byte)'K';
        private const byte RespErr = (byte)'E';

        private const byte CmdEepromDelete = (byte)'D';
        private const int EepromChunkSize = 64;

        public Form1()
        {
            InitializeComponent();

            openFileDialog1.Filter = "Binary files (*.bin)|*.bin|All files (*.*)|*.*";
            openFileDialog1.Title = "Выберите BIN файл";

            lblStatus.Text = "Ready";

            btnRefreshPorts.Click += btnRefreshPorts_Click;
            btnBrowse.Click += btnBrowse_Click;
            btnStart.Click += btnStart_Click;

            //btnSaveToEeprom.Click += btnSaveToEeprom_Click;
            //btnCountFirmware.Click += btnCountFirmware_Click;
            //btnFlashFromEeprom.Click += btnFlashFromEeprom_Click;
            //btnClearLog.Click += btnClearLog_Click;

            RefreshPorts();
        }

        private void RefreshPorts()
        {
            string[] ports = SerialPort.GetPortNames()
                .OrderBy(p => p)
                .ToArray();

            cbPorts.Items.Clear();
            cbPorts.Items.AddRange(ports);

            if (ports.Length > 0)
                cbPorts.SelectedIndex = 0;

            Log("Ports: " + (ports.Length == 0 ? "not found" : string.Join(", ", ports)));
        }

        private void btnRefreshPorts_Click(object sender, EventArgs e)
        {
            RefreshPorts();
        }

        private void btnBrowse_Click(object sender, EventArgs e)
        {
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                tbFile.Text = openFileDialog1.FileName;
                Log("Selected file: " + tbFile.Text);
            }
        }

        private async void btnStart_Click(object sender, EventArgs e)
        {
            await StartTransferAsync();
        }

        private async Task StartTransferAsync()
        {
            string filePath = tbFile.Text.Trim();

            if (string.IsNullOrWhiteSpace(filePath) || !File.Exists(filePath))
            {
                MessageBox.Show("Выберите .bin файл.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            SetUiEnabled(false);
            progressBar.Value = 0;
            lblStatus.Text = "Transfer...";

            try
            {
                byte[] fileData = File.ReadAllBytes(filePath);

                using (SerialPort port = OpenSelectedPort())
                {
                    await ProgramTargetAsync(port, fileData);
                }

                Log("Transfer complete");
                lblStatus.Text = "Done";
                MessageBox.Show("Передача завершена успешно.", "Успех", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                Log("ERROR: " + ex.Message);
                lblStatus.Text = "Error";
                MessageBox.Show(ex.Message, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                SetUiEnabled(true);
            }
        }

        private SerialPort OpenSelectedPort()
        {
            string portName = cbPorts.SelectedItem?.ToString();

            if (string.IsNullOrWhiteSpace(portName))
                throw new IOException("Выберите COM-порт.");

            SerialPort port = new SerialPort(portName, BaudRate, Parity.None, 8, StopBits.One);
            port.ReadTimeout = AckTimeoutMs;
            port.WriteTimeout = AckTimeoutMs;

            port.Open();

            if (!port.IsOpen)
                throw new IOException("Не удалось открыть COM-порт.");

            port.DiscardInBuffer();
            port.DiscardOutBuffer();

            Log("Port opened: " + portName + ", " + BaudRate);
            return port;
        }

        private async Task ProgramTargetAsync(SerialPort port, byte[] fileData)
        {
            int totalChunks = (fileData.Length + ChunkSize - 1) / ChunkSize;

            if (totalChunks == 0)
                throw new IOException("Файл пустой.");

            progressBar.Minimum = 0;
            progressBar.Maximum = totalChunks;

            Log("File size: " + fileData.Length + " bytes");
            Log("Chunks: " + totalChunks);

            lblStatus.Text = "Checking target...";
            bool targetOk = await CheckTargetAsync(port);
            if (!targetOk)
                throw new IOException("Плата не подключена к программатору. Подключите устройство и повторите попытку.");

            Log("Target detected");

            lblStatus.Text = "Starting program mode...";
            bool startOk = await StartProgrammingAsync(port, totalChunks);
            if (!startOk)
                throw new IOException("Программатор не подтвердил начало прошивки.");

            Log("Program mode started");

            for (int chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++)
            {
                int offset = chunkIndex * ChunkSize;
                byte[] chunk = new byte[ChunkSize];

                int remain = fileData.Length - offset;
                int copyLen = Math.Min(ChunkSize, remain);

                Array.Copy(fileData, offset, chunk, 0, copyLen);

                if (copyLen < ChunkSize)
                {
                    for (int j = copyLen; j < ChunkSize; j++)
                        chunk[j] = 0xFF;
                }

                bool sent = false;

                for (int attempt = 1; attempt <= MaxRetries; attempt++)
                {
                    port.DiscardInBuffer();

                    Log($"Send block {chunkIndex + 1}/{totalChunks}, try {attempt}: {BitConverter.ToString(chunk)}");

                    port.Write(chunk, 0, chunk.Length);

                    byte[] echo = new byte[4];
                    bool ok = await ReadExactAsync(port, echo, 4, AckTimeoutMs);

                    if (ok)
                    {
                        uint value = BitConverter.ToUInt32(echo, 0);
                        Log($"Response: 0x{value:X8}");
                        sent = true;
                        break;
                    }
                    else
                    {
                        Log("Timeout");
                    }
                }

                if (!sent)
                    throw new IOException("Не удалось передать блок " + (chunkIndex + 1));

                progressBar.Value = chunkIndex + 1;
                lblStatus.Text = $"Sent: {chunkIndex + 1}/{totalChunks}";
                Application.DoEvents();
            }

            await EndProgrammingAsync(port);
        }

        private async Task<bool> CheckTargetAsync(SerialPort port)
        {
            Log("Send command: CHECK_TARGET (C)");
            port.Write(new[] { CmdCheck }, 0, 1);

            byte[] resp = new byte[1];
            bool ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);

            if (!ok)
            {
                Log("CHECK timeout");
                return false;
            }

            Log($"CHECK response: 0x{resp[0]:X2} ({(char)resp[0]})");
            return resp[0] == RespOk;
        }

        private async Task<bool> StartProgrammingAsync(SerialPort port, int totalChunks)
        {
            Log("Send command: START_PROGRAM (S)");
            port.Write(new[] { CmdStart }, 0, 1);

            byte[] sizeBytes = BitConverter.GetBytes((uint)totalChunks);
            Log($"Send word count: {totalChunks}");
            port.Write(sizeBytes, 0, 4);

            byte[] resp = new byte[1];
            bool ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);

            if (!ok)
            {
                Log("START timeout");
                return false;
            }

            Log($"START response: 0x{resp[0]:X2} ({(char)resp[0]})");
            return resp[0] == RespOk;
        }

        private async Task EndProgrammingAsync(SerialPort port)
        {
            Log("Send command: END_PROGRAM (E)");
            port.Write(new[] { CmdEnd }, 0, 1);

            byte[] resp = new byte[1];
            bool ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);

            if (ok)
                Log($"END response: 0x{resp[0]:X2} ({(char)resp[0]})");
            else
                Log("END timeout");
        }

        private async Task<bool> SaveFirmwareToEepromAsync(SerialPort port, byte[] firmware)
        {
            if (firmware == null || firmware.Length == 0)
                throw new IOException("Файл пустой.");

            Log("Send command: EEPROM_SAVE (W)");
            port.Write(new[] { CmdEepromSave }, 0, 1);

            byte[] lengthBytes = BitConverter.GetBytes((uint)firmware.Length);
            Log($"Send firmware length: {firmware.Length}");
            port.Write(lengthBytes, 0, 4);

            byte[] resp = new byte[1];
            bool ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);
            if (!ok)
            {
                Log("EEPROM_SAVE start timeout");
                return false;
            }

            Log($"EEPROM_SAVE start response: 0x{resp[0]:X2} ({(char)resp[0]})");
            if (resp[0] != RespOk)
                return false;

            int totalChunks = (firmware.Length + EepromChunkSize - 1) / EepromChunkSize;

            progressBar.Minimum = 0;
            progressBar.Maximum = totalChunks;
            progressBar.Value = 0;

            for (int i = 0; i < totalChunks; i++)
            {
                int offset = i * EepromChunkSize;
                int len = Math.Min(EepromChunkSize, firmware.Length - offset);

                byte[] chunk = new byte[len];
                Array.Copy(firmware, offset, chunk, 0, len);

                Log($"EEPROM chunk {i + 1}/{totalChunks}, len={len}");
                port.Write(chunk, 0, chunk.Length);

                ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);
                if (!ok)
                {
                    Log("EEPROM chunk timeout");
                    return false;
                }

                Log($"EEPROM chunk response: 0x{resp[0]:X2} ({(char)resp[0]})");
                if (resp[0] != RespOk)
                    return false;

                progressBar.Value = i + 1;
                lblStatus.Text = $"Saved to EEPROM: {i + 1}/{totalChunks}";
                Application.DoEvents();
            }

            ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);
            if (!ok)
            {
                Log("EEPROM_SAVE finish timeout");
                return false;
            }

            Log($"EEPROM_SAVE finish response: 0x{resp[0]:X2} ({(char)resp[0]})");
            return resp[0] == RespOk;
        }

        private async Task<int> ReadFirmwareCountAsync(SerialPort port)
        {
            Log("Send command: EEPROM_COUNT (N)");
            port.Write(new[] { CmdEepromCount }, 0, 1);

            byte[] resp = new byte[4];
            bool ok = await ReadExactAsync(port, resp, 4, AckTimeoutMs);

            if (!ok)
                throw new IOException("Таймаут при чтении количества прошивок.");

            int count = BitConverter.ToInt32(resp, 0);
            Log("Firmware count = " + count);

            if (count < 0)
                throw new IOException("Получено некорректное количество прошивок.");

            return count;
        }

        private async Task<byte[]> ReadFirmwareByIndexAsync(SerialPort port, int index)
        {
            Log($"Send command: EEPROM_READ (R), index={index}");
            port.Write(new[] { CmdEepromRead }, 0, 1);

            byte[] indexBytes = BitConverter.GetBytes((uint)index);
            port.Write(indexBytes, 0, 4);

            byte[] lenBytes = new byte[4];
            bool ok = await ReadExactAsync(port, lenBytes, 4, AckTimeoutMs);
            if (!ok)
                throw new IOException("Не удалось прочитать длину прошивки из EEPROM.");

            int length = BitConverter.ToInt32(lenBytes, 0);
            Log("EEPROM firmware length = " + length);

            if (length <= 0)
                throw new IOException("Некорректная длина прошивки: " + length);

            byte[] data = new byte[length];
            ok = await ReadExactAsync(port, data, length, Math.Max(AckTimeoutMs, length * 2));

            if (!ok)
                throw new IOException("Не удалось прочитать данные прошивки из EEPROM.");

            Log("EEPROM firmware read complete");
            return data;
        }

        private async Task RefreshFirmwareListAsync()
        {
            using (SerialPort port = OpenSelectedPort())
            {
                int count = await ReadFirmwareCountAsync(port);

                cbFirmwareList.Items.Clear();

                for (int i = 0; i < count; i++)
                    cbFirmwareList.Items.Add("Прошивка " + (i + 1));

                if (count > 0)
                    cbFirmwareList.SelectedIndex = 0;

                Log("Firmware list updated, count = " + count);
            }
        }

        private static async Task<bool> ReadExactAsync(SerialPort port, byte[] buffer, int length, int timeoutMs)
        {
            return await Task.Run(() =>
            {
                int read = 0;
                DateTime start = DateTime.Now;

                while (read < length)
                {
                    if ((DateTime.Now - start).TotalMilliseconds > timeoutMs)
                        return false;

                    try
                    {
                        int n = port.Read(buffer, read, length - read);
                        if (n > 0)
                            read += n;
                    }
                    catch (TimeoutException)
                    {
                        return false;
                    }
                    catch
                    {
                        return false;
                    }

                    Thread.Sleep(1);
                }

                return true;
            });
        }

        private void SetUiEnabled(bool enabled)
        {
            btnStart.Enabled = enabled;
            btnBrowse.Enabled = enabled;
            btnRefreshPorts.Enabled = enabled;
            btnSaveToEeprom.Enabled = enabled;
            btnCountFirmware.Enabled = enabled;
            btnFlashFromEeprom.Enabled = enabled;
            btnClearLog.Enabled = enabled;

            cbPorts.Enabled = enabled;
            cbFirmwareList.Enabled = enabled;
        }

        private void Log(string text)
        {
            tbLog.AppendText($"[{DateTime.Now:HH:mm:ss}] {text}{Environment.NewLine}");
        }

        private async void btnSaveToEeprom_Click(object sender, EventArgs e)
        {
            string filePath = tbFile.Text.Trim();

            if (string.IsNullOrWhiteSpace(filePath) || !File.Exists(filePath))
            {
                MessageBox.Show("Выберите .bin файл.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            SetUiEnabled(false);
            progressBar.Value = 0;
            lblStatus.Text = "Saving to EEPROM...";

            try
            {
                byte[] firmware = File.ReadAllBytes(filePath);

                using (SerialPort port = OpenSelectedPort())
                {
                    bool ok = await SaveFirmwareToEepromAsync(port, firmware);

                    if (!ok)
                        throw new IOException("Не удалось сохранить прошивку в EEPROM.");
                }

                using (SerialPort port = OpenSelectedPort())
                {
                    int count = await ReadFirmwareCountAsync(port);

                    cbFirmwareList.Items.Clear();
                    for (int i = 0; i < count; i++)
                        cbFirmwareList.Items.Add("Прошивка " + (i + 1));

                    if (count > 0)
                        cbFirmwareList.SelectedIndex = count - 1;
                }

                lblStatus.Text = "Saved to EEPROM";
                MessageBox.Show("Прошивка сохранена в EEPROM.", "Успех", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                Log("ERROR: " + ex.Message);
                lblStatus.Text = "Error";
                MessageBox.Show(ex.Message, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                SetUiEnabled(true);
            }
        }

        private async void btnCountFirmware_Click(object sender, EventArgs e)
        {
            SetUiEnabled(false);
            lblStatus.Text = "Reading count...";

            try
            {
                using (SerialPort port = OpenSelectedPort())
                {
                    int count = await ReadFirmwareCountAsync(port);

                    cbFirmwareList.Items.Clear();
                    for (int i = 0; i < count; i++)
                        cbFirmwareList.Items.Add("Прошивка " + (i + 1));

                    if (count > 0)
                        cbFirmwareList.SelectedIndex = 0;

                    lblStatus.Text = $"Firmware count: {count}";
                    MessageBox.Show("Количество прошивок в EEPROM: " + count, "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
            }
            catch (Exception ex)
            {
                Log("ERROR: " + ex.Message);
                lblStatus.Text = "Error";
                MessageBox.Show(ex.Message, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                SetUiEnabled(true);
            }
        }

        private async void btnFlashFromEeprom_Click(object sender, EventArgs e)
        {
            if (cbFirmwareList.Items.Count == 0)
            {
                MessageBox.Show("Список прошивок пуст. Сначала нажмите кнопку подсчёта прошивок.", "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            if (cbFirmwareList.SelectedIndex < 0)
            {
                MessageBox.Show("Выберите прошивку из списка.", "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            SetUiEnabled(false);
            progressBar.Value = 0;
            lblStatus.Text = "Reading from EEPROM...";

            try
            {
                int index = cbFirmwareList.SelectedIndex;

                using (SerialPort port = OpenSelectedPort())
                {
                    byte[] firmware = await ReadFirmwareByIndexAsync(port, index);

                    port.DiscardInBuffer();
                    port.DiscardOutBuffer();

                    lblStatus.Text = "Flashing target...";
                    await ProgramTargetAsync(port, firmware);
                }

                Log("Flash from EEPROM complete");
                lblStatus.Text = "Done";
                MessageBox.Show("Плата прошита выбранной прошивкой из EEPROM.", "Успех", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                Log("ERROR: " + ex.Message);
                lblStatus.Text = "Error";
                MessageBox.Show(ex.Message, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                SetUiEnabled(true);
            }
        }

        private void btnClearLog_Click(object sender, EventArgs e)
        {
            tbLog.Clear();
            Log("Log cleared");
        }

        private async Task<bool> DeleteFirmwareAsync(SerialPort port, int index)
        {
            Log($"Send command: EEPROM_DELETE (D), index={index}");

            port.Write(new[] { CmdEepromDelete }, 0, 1);

            byte[] indexBytes = BitConverter.GetBytes((uint)index);
            port.Write(indexBytes, 0, 4);

            byte[] resp = new byte[1];
            bool ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);

            if (!ok)
            {
                Log("DELETE timeout");
                return false;
            }

            Log($"DELETE response: 0x{resp[0]:X2} ({(char)resp[0]})");

            return resp[0] == RespOk;
        }

        private async void buttonDeleteFromEeprom_Click(object sender, EventArgs e)
        {
            if (cbFirmwareList.Items.Count == 0)
            {
                MessageBox.Show("Список пуст.", "Информация",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            if (cbFirmwareList.SelectedIndex < 0)
            {
                MessageBox.Show("Выберите прошивку.", "Информация",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            int index = cbFirmwareList.SelectedIndex;

            var confirm = MessageBox.Show(
                $"Удалить прошивку {index + 1}?",
                "Подтверждение",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Question);

            if (confirm != DialogResult.Yes)
                return;

            SetUiEnabled(false);
            lblStatus.Text = "Deleting...";

            try
            {
                using (SerialPort port = OpenSelectedPort())
                {
                    bool ok = await DeleteFirmwareAsync(port, index);

                    if (!ok)
                        throw new IOException("Не удалось удалить прошивку.");
                }

                // обновляем список после удаления
                await RefreshFirmwareListAsync();

                lblStatus.Text = "Deleted";
                MessageBox.Show("Прошивка удалена.", "Успех",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                Log("ERROR: " + ex.Message);
                lblStatus.Text = "Error";
                MessageBox.Show(ex.Message, "Ошибка",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            finally
            {
                SetUiEnabled(true);
            }
        }
    }
}