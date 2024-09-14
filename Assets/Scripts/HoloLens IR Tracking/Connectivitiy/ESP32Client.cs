using System;

using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
#if WINDOWS_UWP 

using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Devices.Enumeration;
using Windows.Storage.Streams;
#endif
public class ESP32Client
        {
#if WINDOWS_UWP 

    protected BluetoothLEDevice espServer;
            protected GattDeviceService service;
            public GattCharacteristic characteristic;
            public DeviceWatcher deviceWatcher;

            Stopwatch watch = new Stopwatch();

            public ESP32Client()
            {
                Initialize();
            }
            protected virtual void Initialize()
            {
                // Query for extra properties you want returned
                string[] requestedProperties = { "System.Devices.Aep.DeviceAddress", "System.Devices.Aep.IsConnected" };

                DeviceWatcher deviceWatcher =
                                DeviceInformation.CreateWatcher(
                                        BluetoothLEDevice.GetDeviceSelectorFromPairingState(false),
                                        requestedProperties,
                                        DeviceInformationKind.AssociationEndpoint);

                // Register event handlers before starting the watcher.
                // Added, Updated and Removed are required to get all nearby devices
                deviceWatcher.Added += DeviceWatcher_Added;
                deviceWatcher.Updated += DeviceWatcher_Updated;
                deviceWatcher.Removed += DeviceWatcher_Removed;

                // EnumerationCompleted and Stopped are optional to implement.
                //deviceWatcher.EnumerationCompleted += DeviceWatcher_EnumerationCompleted;
                //deviceWatcher.Stopped += DeviceWatcher_Stopped;

                // Start the watcher.
                deviceWatcher.Start();
            }
            private void DeviceWatcher_Added(DeviceWatcher sender, DeviceInformation args)
            {
            }


            private void DeviceWatcher_Updated(DeviceWatcher sender, DeviceInformationUpdate args)
            {
            }

            private void DeviceWatcher_Removed(DeviceWatcher sender, DeviceInformationUpdate args)
            {
            }


            public async Task<GattCharacteristic> Connect(string mac, string uuid)
            {
                string macAddr = mac.Replace(":", "");
                UInt64 addr = Convert.ToUInt64(macAddr, 16);
                 Debug.WriteLine("Connect");
                espServer = await BluetoothLEDevice.FromBluetoothAddressAsync(addr);
                                 Debug.WriteLine("Connect2");

                //GattDeviceServicesResult result = await espServer.GetGattServicesAsync();
                                 //Debug.WriteLine("Connect3");
                GattDeviceServicesResult result = await espServer.GetGattServicesForUuidAsync(new Guid(uuid));
                Debug.WriteLine(result.Services.Count);

                service = result.Services[0];

                GattCharacteristicsResult res = await service.GetCharacteristicsAsync();

                foreach (GattCharacteristic c in res.Characteristics)
                {
                    GattCharacteristicProperties properties = c.CharacteristicProperties;

                    if (properties.HasFlag(GattCharacteristicProperties.Notify))
                    {
                        characteristic = c;
                        GattCommunicationStatus status = await characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                                GattClientCharacteristicConfigurationDescriptorValue.Notify);

                    }
                }
                //characteristic.ValueChanged += Notify;
                Debug.WriteLine("Test");
                return characteristic;
            }
            protected virtual void Notify(GattCharacteristic c, GattValueChangedEventArgs a)
            {
                var reader = DataReader.FromBuffer(a.CharacteristicValue);
                reader.ByteOrder = ByteOrder.LittleEndian;
                //Debug.WriteLine(reader.UnconsumedBufferLength);
                ulong dt = reader.ReadUInt64();
                //Debug.WriteLine(dt);

                //Debug.WriteLine(watch.ElapsedTicks / ((double)TimeSpan.TicksPerSecond));
                watch.Restart();

                //var reader = DataReader.FromBuffer(a.CharacteristicValue);
                //reader.ByteOrder = ByteOrder.LittleEndian;
                //double ax = reader.ReadDouble();
                //double ay = reader.ReadDouble();
                //double az = reader.ReadDouble();
                //double gx = reader.ReadDouble();
                //double gy = reader.ReadDouble();
                //double gz = reader.ReadDouble();
                //double mx = reader.ReadDouble();
                //double my = reader.ReadDouble();
                //double mz = reader.ReadDouble();

                //Debug.WriteLine(ax + " " + ay + " " + az);

            }
#endif
}
