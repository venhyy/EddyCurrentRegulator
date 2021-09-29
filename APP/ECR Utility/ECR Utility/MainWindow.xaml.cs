using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using System.Collections.ObjectModel;
using Newtonsoft.Json;

namespace ECR_Utility
{
    /// <summary>
    /// Interakční logika pro MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        System.IO.Ports.SerialPort serialPort = null;
        PID pid = new PID();
        public MainWindow()
        {
            InitializeComponent();
            SelPort();
        }

        public class ComPort
        {
            public string DeviceID { get; set; }
            public string Description { get; set; }
        }

        public class PID
        {
            public int Setpoint { get; set; }
            public int Kp { get; set; }
            public int Ki { get; set; }
            public int Kd { get; set; }
          
        }

        private void SelPort()
        {
            // Get serial-port name
            string[] PortList = SerialPort.GetPortNames();
            var MyList = new ObservableCollection<ComPort>();
            foreach (string p in PortList)
            {
                System.Console.WriteLine(p);
                MyList.Add(new ComPort { DeviceID = p, Description = p });
            }

            cmb.ItemsSource = MyList;
            cmb.SelectedValuePath = "DeviceID";
            cmb.DisplayMemberPath = "Description";
        }

        private void connectSerialBtn_Click(object sender, RoutedEventArgs e)
        {
            // When selected serial-port
            if (cmb.SelectedValue != null)
            {
                // Get serial-port name
                var port = cmb.SelectedValue.ToString();
                // When disconnected
                if (serialPort == null)
                {
                    // Setting serial-port
                    serialPort = new SerialPort
                    {
                        PortName = port,
                        BaudRate = 9600,
                        DataBits = 8,
                        Parity = Parity.None,
                        StopBits = StopBits.One,
                        Encoding = Encoding.UTF8,
                        WriteTimeout = 100000
                    };

                    // When received data
                    serialPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(SerialPort_DataReceived);

                    // Try to connect
                    try
                    {
                        // Open serial-port
                        serialPort.Open();
                        if (serialPort.IsOpen)
                        {
                            MessageBox.Show("Připojeno", "ECR Utility", MessageBoxButton.OK, MessageBoxImage.Information);
                        }

                        serialPort.WriteLine("E-10-ACK");



                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message);
                    }
                }
            }
        }

        private void SerialPort_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            // Check connected or disconnected
            if (serialPort == null) return;
            if (serialPort.IsOpen == false) return;

            // Show in the textbox
            try
            {
                var inp = serialPort.ReadLine();
                Console.WriteLine(inp);
                /*channel1Value.Dispatcher.Invoke(
                    new Action(() =>
                    {

                        string str = serialPort.ReadLine();
                        if (str.Contains("Hz"))
                        {
                            string[] subs = str.Split(' ');
                            //channel1Value.Content = subs[0] + " Hz";
                        }
                        // receiveText.AppendText(serialPort.ReadLine());


                    })
                );*/
            }
            catch (Exception exp)
            {
                /*  receiveText.Dispatcher.Invoke(
                      new Action(() =>
                      {
                          receiveText.Text = "!Error! cannot connect" + serialPort.PortName;
                      })
                  );*/

                MessageBox.Show(exp.ToString());
            }
        }

        private void setParameters_Click(object sender, RoutedEventArgs e)
        {
            if (serialPort != null && serialPort.IsOpen) {

                pid.Setpoint = Convert.ToInt32(setpoint.Text);
                pid.Kp = Convert.ToInt32(kp.Text);
                pid.Ki = Convert.ToInt32(ki.Text);
                pid.Kd = Convert.ToInt32(kd.Text);

                string json = JsonConvert.SerializeObject(pid);

                serialPort.WriteLine(json);
            }

                
            
        }
    }


    }
    

