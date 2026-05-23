using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Text;
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
        private const byte CmdEepromList = (byte)'L';
        private const byte CmdEepromSave = (byte)'W';
        private const byte CmdEepromCount = (byte)'N';
        private const byte CmdEepromRead = (byte)'R';
        private const byte CmdEepromInfo = (byte)'I';
        private const byte RespOk = (byte)'K';
        private const byte RespErr = (byte)'E';
        private const byte CmdEepromVerify = (byte)'V';
        private const byte CmdEepromDelete = (byte)'D';
        private const byte CmdEepromActivate = (byte)'A';
        private const byte CmdEepromFlashToTarget = (byte)'F';
        private const int EepromChunkSize = 64;
        private const int FirmwareNameMaxLength = 31;

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

                string defaultName = Path.GetFileNameWithoutExtension(tbFile.Text);
                if (defaultName.Length > FirmwareNameMaxLength)
                    defaultName = defaultName.Substring(0, FirmwareNameMaxLength);

                tbFirmwareName.Text = defaultName;

                Log("Selected file: " + tbFile.Text);
                Log("Firmware name: " + tbFirmwareName.Text);
            }
        }

        private async void btnStart_Click(object sender, EventArgs e)
        {
            await StartTransferAsync();
        }

        private async void btnFlashFileSwd_Click(object sender, EventArgs e)
        {
            await FlashFileBySwdAsync();
        }

        private async Task FlashFileBySwdAsync()
        {
            string filePath = tbFile.Text.Trim();

            if (string.IsNullOrWhiteSpace(filePath) || !File.Exists(filePath))
            {
                MessageBox.Show("Выберите .bin файл.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            SetUiEnabled(false);
            progressBar.Value = 0;
            lblStatus.Text = "Flashing by SWD...";

            try
            {
                byte[] fileData = File.ReadAllBytes(filePath);

                using (SerialPort port = OpenSelectedPort())
                {
                    await ProgramTargetAsync(port, fileData);
                }

                Log("Direct SWD flash complete");
                lblStatus.Text = "SWD flash done";
                MessageBox.Show("Файл прошит напрямую по SWD.", "Успех", MessageBoxButtons.OK, MessageBoxIcon.Information);
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
            lblStatus.Text = "Saving to AT25...";

            try
            {
                byte[] fileData = File.ReadAllBytes(filePath);

                using (SerialPort port = OpenSelectedPort())
                {
                    bool ok = await SaveFirmwareToEepromAsync(port, fileData, filePath);
                    if (!ok)
                        throw new IOException("Не удалось сохранить прошивку в AT25.");
                }

                await RefreshFirmwareListAsync();

                Log("Save to AT25 complete");
                lblStatus.Text = "Saved to AT25";
                MessageBox.Show("Прошивка сохранена во внешней памяти AT25.", "Успех",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
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
            bool ok = await ReadExactAsync(port, resp, 1, 5000);

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
            bool ok = await ReadExactAsync(port, resp, 1, 20000);

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

        private async Task<bool> SaveFirmwareToEepromAsync(SerialPort port, byte[] firmware, string filePath)
        {
            if (firmware == null || firmware.Length == 0)
                throw new IOException("Файл пустой.");

            Log("Send command: EEPROM_SAVE (W)");
            port.Write(new[] { CmdEepromSave }, 0, 1);

            // Ждем готовности к приему размера
            byte[] readyResp = new byte[1];
            bool ok = await ReadExactAsync(port, readyResp, 1, 5000);
            if (!ok || readyResp[0] != RespOk)
            {
                Log("EEPROM_SAVE not ready");
                return false;
            }

            string firmwareName = GetFirmwareNameOrThrow(filePath);
            uint firmwareCrc = CalculateCrc32(firmware);
            byte[] headerBytes = BuildFirmwareTransferHeader(firmware.Length, 1, firmwareName, firmwareCrc);

            Log($"Send firmware header: size={firmware.Length}, version=1, name='{firmwareName}', crc=0x{firmwareCrc:X8}");
            port.Write(headerBytes, 0, headerBytes.Length);

            // Ждем подтверждение получения заголовка
            ok = await ReadExactAsync(port, readyResp, 1, 10000);
            if (!ok || readyResp[0] != RespOk)
            {
                Log("EEPROM_SAVE header not accepted");
                return false;
            }

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

                // Ждем подтверждение каждого чанка
                ok = await ReadExactAsync(port, readyResp, 1, 10000);
                if (!ok || readyResp[0] != RespOk)
                {
                    Log($"EEPROM chunk {i + 1} failed");
                    return false;
                }

                progressBar.Value = i + 1;
                lblStatus.Text = $"Saved to EEPROM: {i + 1}/{totalChunks}";
                Application.DoEvents();
            }

            // Ждем финального подтверждения
            ok = await ReadExactAsync(port, readyResp, 1, 10000);
            if (!ok || readyResp[0] != RespOk)
            {
                Log("EEPROM_SAVE final confirmation failed");
                return false;
            }

            Log("EEPROM_SAVE completed successfully");
            return true;
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
                await ReadFirmwareListWithNamesAsync(port, 0);
            }
        }

        private async Task<int> ReadFirmwareListWithNamesAsync(SerialPort port, int preferredIndex)
        {
            Log("Send command: EEPROM_LIST (L)");
            port.DiscardInBuffer();
            port.Write(new[] { CmdEepromList }, 0, 1);

            byte[] buffer = new byte[2048];
            int read = 0;
            DateTime start = DateTime.Now;
            int quietMs = 0;

            while ((DateTime.Now - start).TotalMilliseconds < 2500 && read < buffer.Length)
            {
                int before = read;

                try
                {
                    while (port.BytesToRead > 0 && read < buffer.Length)
                        buffer[read++] = (byte)port.ReadByte();
                }
                catch { }

                if (read > before)
                    quietMs = 0;
                else
                    quietMs += 20;

                if (read > 0 && quietMs >= 200)
                    break;

                await Task.Delay(20);
            }

            string text = Encoding.ASCII.GetString(buffer, 0, read);
            if (!string.IsNullOrWhiteSpace(text))
                Log(text.Trim());

            cbFirmwareList.Items.Clear();
            string[] lines = text.Split(
                new[] { "\r\n", "\n" },
                StringSplitOptions.RemoveEmptyEntries);

            foreach (string rawLine in lines)
            {
                string line = rawLine.Trim();
                if (line.Length == 0)
                    continue;

                string item = ParseFirmwareListLine(line);
                if (!string.IsNullOrWhiteSpace(item))
                    cbFirmwareList.Items.Add(item);
            }

            int count = cbFirmwareList.Items.Count;

            if (count > 0)
            {
                if (preferredIndex < 0)
                    preferredIndex = 0;
                if (preferredIndex >= count)
                    preferredIndex = count - 1;

                cbFirmwareList.SelectedIndex = preferredIndex;
            }

            Log("Firmware list updated, count = " + count);
            return count;
        }

        private static string ParseFirmwareListLine(string line)
        {
            // STM32 sends lines like:
            // Firmware #1: MyFirmware v1 (4096 bytes)
            int colon = line.IndexOf(':');
            if (colon >= 0 && colon + 1 < line.Length)
                return line.Substring(colon + 1).Trim();

            return line.Trim();
        }
        private async Task<bool> VerifyFirmwareAsync(SerialPort port, int index)
        {
            Log($"Send command: EEPROM_VERIFY (V), index={index}");

            port.DiscardInBuffer();

            port.Write(new[] { CmdEepromVerify }, 0, 1);

            byte[] indexBytes = BitConverter.GetBytes((uint)index);
            port.Write(indexBytes, 0, 4);

            // Ждём быстрый ответ: STM32 приняла команду или нет
            byte[] resp = new byte[1];
            bool ok = await ReadExactAsync(port, resp, 1, 3000);

            if (!ok)
                throw new IOException("STM32 не ответил на команду VERIFY.");

            Log($"VERIFY ack: 0x{resp[0]:X2} ({(char)resp[0]})");

            if (resp[0] != RespOk)
                throw new IOException("STM32 отклонил команду VERIFY.");

            // CRC может считаться долго на больших файлах
            byte[] crcBytes = new byte[4];
            ok = await ReadExactAsync(port, crcBytes, 4, 30000);

            if (!ok)
                throw new IOException("Таймаут при чтении CRC.");

            uint crc = BitConverter.ToUInt32(crcBytes, 0);

            Log($"VERIFY CRC=0x{crc:X8}");

            return true;
        }
        private async Task<string> ReadMemoryInfoAsync(SerialPort port)
        {
            Log("Send command: EEPROM_INFO (I)");
            port.Write(new[] { CmdEepromInfo }, 0, 1);

            byte[] buffer = new byte[256];
            int read = 0;
            DateTime start = DateTime.Now;

            while ((DateTime.Now - start).TotalMilliseconds < 1500)
            {
                try
                {
                    while (port.BytesToRead > 0 && read < buffer.Length)
                        buffer[read++] = (byte)port.ReadByte();
                }
                catch { }

                await Task.Delay(10);
            }

            string text = System.Text.Encoding.ASCII.GetString(buffer, 0, read);
            Log(text.Trim());

            return text;
        }

        private async Task<bool> FlashFirmwareFromAt25Async(SerialPort port, int index)
        {
            Log($"Send command: EEPROM_FLASH_TO_TARGET (F), index={index}");

            port.Write(new[] { CmdEepromFlashToTarget }, 0, 1);

            byte[] indexBytes = BitConverter.GetBytes((uint)index);
            port.Write(indexBytes, 0, 4);

            byte[] resp = new byte[1];

            // STM32 сначала готовит SWD target: reset, init debug, erase flash.
            // Это может занимать несколько секунд.
            bool ok = await ReadExactAsync(port, resp, 1, 20000);
            if (!ok)
                throw new IOException("Таймаут при подготовке прошивки из AT25.");

            if (resp[0] != RespOk)
            {
                Log($"EEPROM_FLASH_TO_TARGET prepare response: 0x{resp[0]:X2} ({(char)resp[0]})");
                return false;
            }

            byte[] sizeBytes = new byte[4];
            ok = await ReadExactAsync(port, sizeBytes, 4, 5000);
            if (!ok)
                throw new IOException("Не удалось прочитать размер прошивки из AT25.");

            int size = BitConverter.ToInt32(sizeBytes, 0);
            if (size <= 0)
                throw new IOException("Некорректный размер прошивки в AT25: " + size);

            int totalChunks = (size + EepromChunkSize - 1) / EepromChunkSize;

            progressBar.Minimum = 0;
            progressBar.Maximum = totalChunks;
            progressBar.Value = 0;

            Log($"AT25 firmware size = {size} bytes");
            Log($"AT25 flash chunks = {totalChunks}");

            for (int i = 0; i < totalChunks; i++)
            {
                ok = await ReadExactAsync(port, resp, 1, 20000);
                if (!ok)
                    throw new IOException("Таймаут при прошивке чанка из AT25: " + (i + 1));

                if (resp[0] != RespOk)
                {
                    Log($"AT25 flash chunk {i + 1} response: 0x{resp[0]:X2} ({(char)resp[0]})");
                    return false;
                }

                progressBar.Value = i + 1;
                lblStatus.Text = $"Flashing from AT25: {i + 1}/{totalChunks}";
                Log($"AT25 flash chunk {i + 1}/{totalChunks} OK");
                Application.DoEvents();
            }

            ok = await ReadExactAsync(port, resp, 1, 20000);
            if (!ok)
                throw new IOException("Таймаут финального подтверждения прошивки из AT25.");

            Log($"EEPROM_FLASH_TO_TARGET final response: 0x{resp[0]:X2} ({(char)resp[0]})");

            return resp[0] == RespOk;
        }

        private string GetFirmwareNameOrThrow(string filePath)
        {
            string name = tbFirmwareName.Text.Trim();

            if (string.IsNullOrWhiteSpace(name))
                name = Path.GetFileNameWithoutExtension(filePath);

            if (string.IsNullOrWhiteSpace(name))
                throw new IOException("Введите имя прошивки.");

            if (name.Length > FirmwareNameMaxLength)
                throw new IOException($"Имя прошивки слишком длинное. Максимум {FirmwareNameMaxLength} символ.");

            foreach (char ch in name)
            {
                if (ch < 32 || ch > 126)
                    throw new IOException("Имя прошивки должно содержать только ASCII символы: латиница, цифры, _, -, точка.");
            }

            return name;
        }

        private static byte[] BuildFirmwareTransferHeader(int firmwareSize, uint version, string firmwareName, uint crc32)
        {
            byte[] header = new byte[44];

            Array.Copy(BitConverter.GetBytes((uint)firmwareSize), 0, header, 0, 4);
            Array.Copy(BitConverter.GetBytes(version), 0, header, 4, 4);

            byte[] nameBytes = Encoding.ASCII.GetBytes(firmwareName);
            if (nameBytes.Length > FirmwareNameMaxLength)
                throw new IOException($"Имя прошивки слишком длинное. Максимум {FirmwareNameMaxLength} байт.");

            Array.Copy(nameBytes, 0, header, 8, nameBytes.Length);

            // header[8 + nameBytes.Length] остается 0, это C-строка с \\0
            Array.Copy(BitConverter.GetBytes(crc32), 0, header, 40, 4);

            return header;
        }

        private static uint CalculateCrc32(byte[] data)
        {
            uint crc = 0xFFFFFFFFu;

            for (int i = 0; i < data.Length; i++)
            {
                crc ^= data[i];

                for (int j = 0; j < 8; j++)
                {
                    if ((crc & 1u) != 0)
                        crc = (crc >> 1) ^ 0xEDB88320u;
                    else
                        crc >>= 1;
                }
            }

            return ~crc;
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
            btnVerifyFirmware.Enabled = enabled;
            btnFlashFileSwd.Enabled = enabled;
            btnMemoryInfo.Enabled = enabled;

            cbPorts.Enabled = enabled;
            cbFirmwareList.Enabled = enabled;
            tbFirmwareName.Enabled = enabled;
        }

        private void Log(string text)
        {
            tbLog.AppendText($"[{DateTime.Now:HH:mm:ss}] {text}{Environment.NewLine}");
        }
        private async void btnMemoryInfo_Click(object sender, EventArgs e)
        {
            SetUiEnabled(false);
            lblStatus.Text = "Reading memory info...";

            try
            {
                using (SerialPort port = OpenSelectedPort())
                {
                    string info = await ReadMemoryInfoAsync(port);

                    if (string.IsNullOrWhiteSpace(info))
                        throw new IOException("STM32 не вернул информацию о памяти.");

                    lblStatus.Text = "Memory info";
                    MessageBox.Show(info, "EEPROM Memory Info",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                }
            }
            catch (Exception ex)
            {
                Log("ERROR: " + ex.Message);
                lblStatus.Text = "Error";
                MessageBox.Show(ex.Message);
            }
            finally
            {
                SetUiEnabled(true);
            }
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
                    bool ok = await SaveFirmwareToEepromAsync(port, firmware, filePath);

                    if (!ok)
                        throw new IOException("Не удалось сохранить прошивку в EEPROM.");
                }

                using (SerialPort port = OpenSelectedPort())
                {
                    await ReadFirmwareListWithNamesAsync(port, int.MaxValue);
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
                int count;
                using (SerialPort port = OpenSelectedPort())
                {
                    count = await ReadFirmwareListWithNamesAsync(port, 0);
                }

                lblStatus.Text = $"Firmware count: {count}";
                MessageBox.Show("Количество прошивок в EEPROM: " + count,
                    "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
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
                MessageBox.Show("Список прошивок пуст. Сначала нажмите кнопку подсчёта прошивок.",
                    "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            if (cbFirmwareList.SelectedIndex < 0)
            {
                MessageBox.Show("Выберите прошивку из списка.",
                    "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            int index = cbFirmwareList.SelectedIndex;

            SetUiEnabled(false);
            progressBar.Value = 0;
            lblStatus.Text = "Flashing from AT25...";

            try
            {
                using (SerialPort port = OpenSelectedPort())
                {
                    bool ok = await FlashFirmwareFromAt25Async(port, index);
                    if (!ok)
                        throw new IOException("Не удалось прошить target из AT25.");
                }

                Log("Flash from AT25 complete");
                lblStatus.Text = "Flashed from AT25";
                MessageBox.Show("Target STM32 прошит выбранной прошивкой из AT25.",
                    "Успех", MessageBoxButtons.OK, MessageBoxIcon.Information);
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

        private async void btnVerifyFirmware_Click(object sender, EventArgs e)
        {
            if (cbFirmwareList.Items.Count == 0)
            {
                MessageBox.Show("Список прошивок пуст. Сначала нажмите кнопку подсчёта прошивок.",
                    "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            if (cbFirmwareList.SelectedIndex < 0)
            {
                MessageBox.Show("Выберите прошивку для проверки CRC.",
                    "Информация", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            SetUiEnabled(false);
            lblStatus.Text = "Verifying CRC...";

            try
            {
                using (SerialPort port = OpenSelectedPort())
                {
                    bool ok = await VerifyFirmwareAsync(port, cbFirmwareList.SelectedIndex);

                    if (!ok)
                        throw new IOException("CRC не совпадает или прошивка повреждена.");
                }

                lblStatus.Text = "CRC OK";
                MessageBox.Show("CRC прошивки корректный.", "CRC",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception ex)
            {
                Log("ERROR: " + ex.Message);
                lblStatus.Text = "CRC ERROR";
                MessageBox.Show(ex.Message, "Ошибка CRC",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
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


        private async Task<bool> ActivateFirmwareAsync(SerialPort port, int index)
        {
            Log($"Send command: EEPROM_ACTIVATE (A), index={index}");

            port.Write(new[] { CmdEepromActivate }, 0, 1);

            byte[] indexBytes = BitConverter.GetBytes((uint)index);
            port.Write(indexBytes, 0, 4);

            byte[] resp = new byte[1];
            bool ok = await ReadExactAsync(port, resp, 1, AckTimeoutMs);

            if (!ok)
            {
                Log("ACTIVATE timeout");
                return false;
            }

            Log($"ACTIVATE response: 0x{resp[0]:X2} ({(char)resp[0]})");
            return resp[0] == RespOk;
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