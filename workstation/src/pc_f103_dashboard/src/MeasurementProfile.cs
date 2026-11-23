using System;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using System.Xml.Serialization;

namespace NeckF103
{
    public sealed class MeasurementProfile
    {
        [DisplayName("配置名称")] public string Name { get; set; }
        [DisplayName("安装与共同方向已核对，按读窗中点估算时间")] public bool Registered { get; set; }
        [Category("安装四元数 w x y z")] public string C0 { get; set; }
        [Category("安装四元数 w x y z")] public string C4 { get; set; }
        [Category("安装四元数 w x y z")] public string C7 { get; set; }
        [Category("本地世界到共同世界 w x y z")] public string WorldC0 { get; set; }
        [Category("本地世界到共同世界 w x y z")] public string WorldC4 { get; set; }
        [Category("本地世界到共同世界 w x y z")] public string WorldC7 { get; set; }
        [DisplayName("低通截止频率（mHz）")] public uint Cutoff { get; set; }
        [DisplayName("持续时间（ms）")] public uint Duration { get; set; }
        [DisplayName("前屈警告（千分之一度）")] public uint FlexWarn { get; set; }
        [DisplayName("前屈危险（千分之一度）")] public uint FlexDanger { get; set; }
        [DisplayName("侧倾警告（千分之一度）")] public uint SideWarn { get; set; }
        [DisplayName("侧倾危险（千分之一度）")] public uint SideDanger { get; set; }
        public MeasurementProfile()
        {
            Name="首次台架"; C0=C4=C7=WorldC0=WorldC4=WorldC7="1 0 0 0";
            Cutoff=500; Duration=10000; FlexWarn=15000; FlexDanger=30000; SideWarn=5000; SideDanger=10000;
        }
        internal double[] Rotations()
        {
            if(string.IsNullOrWhiteSpace(Name)) throw new FormatException("请填写配置名称");
            return new[]{C0,C4,C7,WorldC0,WorldC4,WorldC7}.SelectMany(q=>{
                var values=(q??"").Split(new[]{' ',','},StringSplitOptions.RemoveEmptyEntries)
                    .Select(v=>double.Parse(v,CultureInfo.InvariantCulture)).ToArray();
                if(values.Length!=4) throw new FormatException("每个四元数需填写四个数，按 w x y z 排列");
                return values;
            }).ToArray();
        }
        internal uint[] Settings() { return new[]{Cutoff,Duration,FlexWarn,FlexDanger,SideWarn,SideDanger}; }
        public const string Filename="measurement-profile.xml";
        internal static MeasurementProfile Load(string path)
        {
            using(var f=File.OpenRead(path)) return (MeasurementProfile)new XmlSerializer(typeof(MeasurementProfile)).Deserialize(f);
        }
        internal void Save(string path)
        {
            using(var check=new NativeBridge()) check.SetProfile(this);
            string temp=path+"."+Guid.NewGuid().ToString("N")+".tmp";
            try {
                using(var f=File.Create(temp)) new XmlSerializer(typeof(MeasurementProfile)).Serialize(f,this);
                if(File.Exists(path)) File.Replace(temp,path,null); else File.Move(temp,path);
            } finally { if(File.Exists(temp)) File.Delete(temp); }
        }
        internal static void Edit(DataEngine engine, IWin32Window owner)
        {
            if(engine.Active) throw new InvalidOperationException("请先停止采集，再编辑测量配置");
            string path=Path.Combine(engine.TaskRoot,Filename);
            var profile=File.Exists(path)?Load(path):new MeasurementProfile();
            using(var form=new Form { Text="PC 测量配置",Width=600,Height=580 })
            {
                var grid=new PropertyGrid {Dock=DockStyle.Fill,SelectedObject=profile};
                var save=new Button {Dock=DockStyle.Bottom,Text="保存到电脑，下次启动串口时生效"};
                save.Click+=delegate { try {profile.Save(path); form.Close();} catch(Exception ex){MessageBox.Show(form,ex.Message);} };
                form.Controls.Add(grid); form.Controls.Add(save); form.ShowDialog(owner);
            }
        }
    }
}
