//using System;
//using System.IO;
//using System.IO.Ports;
//using System.Linq;
//using System.Threading;
//using System.Threading.Tasks;
//using System.Windows.Forms;

//namespace SWD
//{
//    public partial class Form1 : Form
//    {
//        private const int BaudRate = 115200;
//        private const int ChunkSize = 4;
//        private const int AckTimeoutMs = 300;
//        private const int MaxRetries = 3;
//        private const byte AckByte = 0x01;

//        public Form1()
//        {
//            InitializeComponent();

//            openFileDialog1.Filter = "Binary files (*.bin)|*.bin|All files (*.*)|*.*";
//            openFileDialog1.Title = "Выберите BIN файл";

//            lblStatus.Text = "Ready";

//            btnRefreshPorts.Click += btnRefreshPorts_Click;
//            btnBrowse.Click += btnBrowse_Click;
//            btnStart.Click += btnStart_Click;

//            RefreshPorts();
//        }

//        private void RefreshPorts()
//        {
//            string[] ports = SerialPort.GetPortNames()
//                .OrderBy(p => p)
//                .ToArray();

//            cbPorts.Items.Clear();
//            cbPorts.Items.AddRange(ports);

//            if (ports.Length > 0)
//                cbPorts.SelectedIndex = 0;

//            Log("Ports: " + (ports.Length == 0 ? "not found" : string.Join(", ", ports)));
//        }

//        private void btnRefreshPorts_Click(object sender, EventArgs e)
//        {
//            RefreshPorts();
//        }

//        private void btnBrowse_Click(object sender, EventArgs e)
//        {
//            if (openFileDialog1.ShowDialog() == DialogResult.OK)
//            {
//                tbFile.Text = openFileDialog1.FileName;
//                Log("Selected file: " + tbFile.Text);
//            }
//        }

//        private async void btnStart_Click(object sender, EventArgs e)
//        {
//            await StartTransferAsync();
//        }

//        private async Task StartTransferAsync()
//        {
//            string portName = cbPorts.SelectedItem?.ToString();
//            string filePath = tbFile.Text.Trim();

//            if (string.IsNullOrWhiteSpace(portName))
//            {
//                MessageBox.Show("Выберите COM-порт.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
//                return;
//            }

//            if (string.IsNullOrWhiteSpace(filePath) || !File.Exists(filePath))
//            {
//                MessageBox.Show("Выберите .bin файл.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
//                return;
//            }

//            SetUiEnabled(false);
//            progressBar.Value = 0;
//            lblStatus.Text = "Transfer...";

//            try
//            {
//                byte[] fileData = File.ReadAllBytes(filePath);
//                int totalChunks = (fileData.Length + ChunkSize - 1) / ChunkSize;

//                if (totalChunks == 0)
//                {
//                    MessageBox.Show("Файл пустой.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
//                    return;
//                }

//                progressBar.Minimum = 0;
//                progressBar.Maximum = totalChunks;

//                Log("File size: " + fileData.Length + " bytes");
//                Log("Chunks: " + totalChunks);

//                using (SerialPort port = new SerialPort(portName, BaudRate, Parity.None, 8, StopBits.One))
//                {
//                    port.ReadTimeout = AckTimeoutMs;
//                    port.WriteTimeout = AckTimeoutMs;

//                    port.Open();

//                    if (!port.IsOpen)
//                        throw new IOException("Не удалось открыть COM-порт.");

//                    port.DiscardInBuffer();
//                    port.DiscardOutBuffer();

//                    Log("Port opened: " + portName + ", " + BaudRate);

//                    for (int chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++)
//                    {
//                        int offset = chunkIndex * ChunkSize;
//                        byte[] chunk = new byte[ChunkSize];

//                        int remain = fileData.Length - offset;
//                        int copyLen = Math.Min(ChunkSize, remain);

//                        Array.Copy(fileData, offset, chunk, 0, copyLen);

//                        if (copyLen < ChunkSize)
//                        {
//                            for (int j = copyLen; j < ChunkSize; j++)
//                                chunk[j] = 0xFF;
//                        }

//                        bool sent = false;

//                        for (int attempt = 1; attempt <= MaxRetries; attempt++)
//                        {
//                            port.DiscardInBuffer();

//                            Log($"Send block {chunkIndex + 1}/{totalChunks}, try {attempt}: {BitConverter.ToString(chunk)}");

//                            port.Write(chunk, 0, chunk.Length);

//                            byte[] ack = new byte[4];
//                            bool ok = await ReadExactAsync(port, ack, 4, AckTimeoutMs);

//                            if (ok)
//                            {
//                                uint value = BitConverter.ToUInt32(ack, 0);
//                                Log($"Response: 0x{value:X8}");
//                                sent = true;
//                                break;
//                            }
//                            else
//                            {
//                                Log("Timeout");
//                            }
//                            //if (ok && ack[0] == AckByte)
//                            //{
//                            //    Log("ACK: 0x" + ack[0].ToString("X2"));
//                            //    sent = true;
//                            //    break;
//                            //}

//                            //if (ok)
//                            //    Log("Wrong response: 0x" + ack[0].ToString("X2"));
//                            //else
//                            //    Log("Timeout");
//                        }

//                        if (!sent)
//                        {
//                            throw new IOException("Не удалось передать блок " + (chunkIndex + 1));
//                        }

//                        progressBar.Value = chunkIndex + 1;
//                        lblStatus.Text = $"Sent: {chunkIndex + 1}/{totalChunks}";
//                        Application.DoEvents();
//                    }
//                }

//                Log("Transfer complete");
//                lblStatus.Text = "Done";
//                MessageBox.Show("Передача завершена успешно.", "Успех", MessageBoxButtons.OK, MessageBoxIcon.Information);
//            }
//            catch (Exception ex)
//            {
//                Log("ERROR: " + ex.Message);
//                lblStatus.Text = "Error";
//                MessageBox.Show(ex.Message, "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Error);
//            }
//            finally
//            {
//                SetUiEnabled(true);
//            }
//        }

//        private static async Task<bool> ReadExactAsync(SerialPort port, byte[] buffer, int length, int timeoutMs)
//        {
//            return await Task.Run(() =>
//            {
//                int read = 0;
//                DateTime start = DateTime.Now;

//                while (read < length)
//                {
//                    if ((DateTime.Now - start).TotalMilliseconds > timeoutMs)
//                        return false;

//                    try
//                    {
//                        int n = port.Read(buffer, read, length - read);
//                        if (n > 0)
//                            read += n;
//                    }
//                    catch (TimeoutException)
//                    {
//                        return false;
//                    }
//                    catch
//                    {
//                        return false;
//                    }

//                    Thread.Sleep(1);
//                }

//                return true;
//            });
//        }

//        private void SetUiEnabled(bool enabled)
//        {
//            btnStart.Enabled = enabled;
//            btnBrowse.Enabled = enabled;
//            btnRefreshPorts.Enabled = enabled;
//            cbPorts.Enabled = enabled;
//        }

//        private void Log(string text)
//        {
//            tbLog.AppendText($"[{DateTime.Now:HH:mm:ss}] {text}{Environment.NewLine}");
//        }
//    }
//}
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

        private const byte RespOk = (byte)'K';
        private const byte RespErr = (byte)'E';

        public Form1()
        {
            InitializeComponent();

            openFileDialog1.Filter = "Binary files (*.bin)|*.bin|All files (*.*)|*.*";
            openFileDialog1.Title = "Выберите BIN файл";

            lblStatus.Text = "Ready";

            btnRefreshPorts.Click += btnRefreshPorts_Click;
            btnBrowse.Click += btnBrowse_Click;
            btnStart.Click += btnStart_Click;

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
            string portName = cbPorts.SelectedItem?.ToString();
            string filePath = tbFile.Text.Trim();

            if (string.IsNullOrWhiteSpace(portName))
            {
                MessageBox.Show("Выберите COM-порт.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

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
                int totalChunks = (fileData.Length + ChunkSize - 1) / ChunkSize;

                if (totalChunks == 0)
                {
                    MessageBox.Show("Файл пустой.", "Ошибка", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }

                progressBar.Minimum = 0;
                progressBar.Maximum = totalChunks;

                Log("File size: " + fileData.Length + " bytes");
                Log("Chunks: " + totalChunks);

                using (SerialPort port = new SerialPort(portName, BaudRate, Parity.None, 8, StopBits.One))
                {
                    port.ReadTimeout = AckTimeoutMs;
                    port.WriteTimeout = AckTimeoutMs;

                    port.Open();

                    if (!port.IsOpen)
                        throw new IOException("Не удалось открыть COM-порт.");

                    port.DiscardInBuffer();
                    port.DiscardOutBuffer();

                    Log("Port opened: " + portName + ", " + BaudRate);

                    // 1. Проверка подключения платы
                    lblStatus.Text = "Checking target...";
                    bool targetOk = await CheckTargetAsync(port);
                    if (!targetOk)
                    {
                        throw new IOException("Плата не подключена к программатору. Подключите устройство и повторите попытку.");
                    }

                    Log("Target detected");

                    // 2. Старт прошивки и передача размера
                    lblStatus.Text = "Starting program mode...";
                    bool startOk = await StartProgrammingAsync(port, totalChunks);
                    if (!startOk)
                    {
                        throw new IOException("Программатор не подтвердил начало прошивки.");
                    }

                    Log("Program mode started");

                    // 3. Передача данных по 4 байта
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
                        {
                            throw new IOException("Не удалось передать блок " + (chunkIndex + 1));
                        }

                        progressBar.Value = chunkIndex + 1;
                        lblStatus.Text = $"Sent: {chunkIndex + 1}/{totalChunks}";
                        Application.DoEvents();
                    }

                    // 4. Команда завершения
                    await EndProgrammingAsync(port);

                    Log("Transfer complete");
                    lblStatus.Text = "Done";
                    MessageBox.Show("Передача завершена успешно.", "Успех", MessageBoxButtons.OK, MessageBoxIcon.Information);
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
            {
                Log($"END response: 0x{resp[0]:X2} ({(char)resp[0]})");
            }
            else
            {
                Log("END timeout");
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
            cbPorts.Enabled = enabled;
        }

        private void Log(string text)
        {
            tbLog.AppendText($"[{DateTime.Now:HH:mm:ss}] {text}{Environment.NewLine}");
        }
    }
}