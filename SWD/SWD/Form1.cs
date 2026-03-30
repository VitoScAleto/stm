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
        private const int AckTimeoutMs = 300;
        private const int MaxRetries = 3;
        private const byte AckByte = 0x01;

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

                            byte[] ack = new byte[4];
                            bool ok = await ReadExactAsync(port, ack, 4, AckTimeoutMs);

                            if (ok)
                            {
                                uint value = BitConverter.ToUInt32(ack, 0);
                                Log($"Response: 0x{value:X8}");
                                sent = true;
                                break;
                            }
                            else
                            {
                                Log("Timeout");
                            }
                            //if (ok && ack[0] == AckByte)
                            //{
                            //    Log("ACK: 0x" + ack[0].ToString("X2"));
                            //    sent = true;
                            //    break;
                            //}

                            //if (ok)
                            //    Log("Wrong response: 0x" + ack[0].ToString("X2"));
                            //else
                            //    Log("Timeout");
                        }

                        if (!sent)
                        {
                            throw new IOException("Не удалось передать блок " + (chunkIndex + 1));
                        }

                        progressBar.Value = chunkIndex + 1;
                        lblStatus.Text = $"Sent: {chunkIndex + 1}/{totalChunks}";
                        Application.DoEvents();
                    }
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