using System;
using System.Drawing;
using System.Windows.Forms;
namespace NeckF103
{
    internal sealed class DeviceControlForm : Form
    {
        private readonly Timer timer=new Timer();
        public DeviceControlForm(DataEngine engine)
        {
            Text="F103C8 板端控制"; ClientSize=new Size(650,320); Font=new Font("Microsoft YaHei UI",9);
            FormBorderStyle=FormBorderStyle.FixedDialog; MaximizeBox=false; MinimizeBox=false;
            FlowLayoutPanel layout=new FlowLayoutPanel {Dock=DockStyle.Fill,Padding=new Padding(12),FlowDirection=FlowDirection.TopDown,WrapContents=false};
            Controls.Add(layout);
            layout.Controls.Add(new Label {AutoSize=true,Text="仅控制串口连接的板子。归零成功以零位状态/ref为准，不以命令应答为准。"});
            FlowLayoutPanel buttons=new FlowLayoutPanel {Width=620,Height=78}; layout.Controls.Add(buttons);
            string[] names={"查询状态","请求归零","取消归零","暂停测量","重新登记/恢复","保存参数到Flash"};
            uint[] ops={1,2,3,4,5,7};
            for(int i=0;i<ops.Length;i++) {
                uint op=ops[i]; Button button=new Button {Text=names[i],AutoSize=true,Height=30};
                button.Click+=delegate { try { engine.DeviceCommand(op); } catch(Exception e) { MessageBox.Show(this,e.Message); } };
                if(op==7 && engine.Pipe.View.Session!=null && (engine.Pipe.View.Session.Registration.Capabilities&64)!=0) button.Enabled=false;
                buttons.Controls.Add(button);
            }
            layout.Controls.Add(new Label {AutoSize=true,Text="参数修改：先暂停测量 → 等待新会话 → 修改 → 按需保存 → 恢复 → 重新归零。"});
            FlowLayoutPanel settings=new FlowLayoutPanel {Width=620,Height=40}; layout.Controls.Add(settings);
            ComboBox key=new ComboBox {Width=220,DropDownStyle=ComboBoxStyle.DropDownList};
            key.Items.AddRange(new object[]{"截止频率（mHz，100–20000）","持续时间（ms，100–120000）","前屈警告阈值（mdeg）","前屈危险阈值（mdeg）","侧倾警告阈值（mdeg）","侧倾危险阈值（mdeg）"}); key.SelectedIndex=0;
            NumericUpDown value=new NumericUpDown {Width=120,Minimum=1,Maximum=120000,Value=500};
            Button read=new Button {Text="读取参数",AutoSize=true},apply=new Button {Text="应用参数",AutoSize=true};
            read.Click+=delegate { try { engine.DeviceCommand(1,(uint)(key.SelectedIndex+1)); } catch(Exception e) { MessageBox.Show(this,e.Message); } };
            apply.Click+=delegate { try { engine.DeviceCommand(6,(uint)(key.SelectedIndex+1),(uint)value.Value); } catch(Exception e) { MessageBox.Show(this,e.Message); } };
            settings.Controls.AddRange(new Control[]{key,value,read,apply});
            if(engine.Pipe.View.Session!=null && (engine.Pipe.View.Session.Registration.Capabilities&64)!=0) { settings.Enabled=false; ((Label)layout.Controls[2]).Text="当前采集固件：停止后打开主窗口「PC测量配置」修改和保存参数。"; }
            Label state=new Label {Width=610,Height=90,Text="等待控制登记"}; layout.Controls.Add(state);
            timer.Interval=200; timer.Tick+=delegate { lock(engine.Pipe.Sync) state.Text=engine.Device.Status; };
            Shown+=delegate { timer.Start(); }; FormClosed+=delegate { timer.Dispose(); };
        }
    }
}
