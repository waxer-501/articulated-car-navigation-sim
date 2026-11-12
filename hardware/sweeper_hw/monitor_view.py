"""Tk desktop view. Only monitor/read controls; no board reset or motor controls."""
from collections import deque
import json
import math
import subprocess
import time
import tkinter as tk
from tkinter import ttk

BG='#0b1220'; PANEL='#111e30'; EDGE='#24364d'; FG='#e3ecf5'; MUTED='#96a9bd'
TEAL='#49dcc1'; AMBER='#ffc56b'; PURPLE='#bd9aff'; COLORS=('#ffad6b','#56d9ae','#76baff')
FONT='Noto Sans CJK SC'; MONO='DejaVu Sans Mono'


def label(parent,text='',size=10,color=FG,**kw):
    return tk.Label(parent,text=text,bg=parent.cget('bg'),fg=color,font=(FONT,size),**kw)


class Plot(tk.Canvas):
    def __init__(self,parent,title,key,unit,names):
        super().__init__(parent,bg=PANEL,highlightthickness=1,highlightbackground=EDGE,height=142)
        self.title,self.key,self.unit,self.names=title,key,unit,names
        self.points=[];self.now=0
        self.bind('<Configure>',lambda e:self.draw())

    def update_data(self,points,now):
        self.points,self.now=points,now;self.draw()

    def draw(self):
        self.delete('all');w,h=self.winfo_width(),self.winfo_height()
        if w<100 or h<70:return
        self.create_text(12,15,text=self.title+'  '+self.unit,anchor='w',fill=FG,font=(FONT,10,'bold'))
        points=[(t,r) for t,r in self.points if self.now-60<=t<=self.now+.01]
        for i,name in enumerate(self.names):
            value=f"{points[-1][1][self.key][i]:+.2f}" if points else '—'
            self.create_text(w-290+i*98,15,text=name+' '+value,fill=COLORS[i],font=(MONO,8))
        left,right,top,bottom=58,w-15,38,h-22
        for i in range(4):
            y=top+(bottom-top)*i/3
            self.create_line(left,y,right,y,fill=EDGE)
        if not points:
            self.create_text(w/2,(top+bottom)/2,text='等待当前来源的数据',fill=MUTED,font=(FONT,11));return
        values=[v for _,r in points for v in r[self.key]]
        lo,hi=min(values),max(values)
        pad=max((hi-lo)*.12, .05 if self.key=='euler_deg' else .01)
        lo-=pad;hi+=pad
        t0=max(self.now-60,points[0][0]);t1=max(self.now,t0+1)
        for i in range(4):
            value=hi-(hi-lo)*i/3;y=top+(bottom-top)*i/3
            self.create_text(left-7,y,text=f'{value:.2f}',anchor='e',fill=MUTED,font=(MONO,8))
        for axis in range(3):
            path=[];old_boot=None;old_t=None
            for t,row in points:
                x=left+(right-left)*(t-t0)/(t1-t0)
                y=bottom-(bottom-top)*(row[self.key][axis]-lo)/(hi-lo)
                if path and (row.get('boot_id')!=old_boot or t-old_t>3):
                    if len(path)>=4:self.create_line(*path,fill=COLORS[axis],width=1.8)
                    path=[]
                path.extend((x,y));old_boot=row.get('boot_id');old_t=t
            if len(path)>=4:self.create_line(*path,fill=COLORS[axis],width=1.8)
            if path:self.create_oval(path[-2]-2,path[-1]-2,path[-2]+2,path[-1]+2,fill=COLORS[axis],outline='')
        self.create_text(left,bottom+12,text=f'−{self.now-t0:.0f}s',anchor='w',fill=MUTED,font=(MONO,8))
        self.create_text(right,bottom+12,text='当前显示时刻',anchor='e',fill=MUTED,font=(FONT,8))


class MonitorView:
    def __init__(self,root,state,hardware,folder,archive,offline,error):
        self.root,self.state,self.hardware,self.folder=root,state,hardware,folder
        self.archive,self.offline=archive,offline
        self.playing=False;self.play_cursor=len(archive);self.play_start=0;self.play_speed=1;self.replay_base=0
        self.replay_time=archive[-1]['host_received_monotonic_s'] if archive else 0
        self.frame=0
        root.title('铰接小车 · 双 IMU 通信监视')
        root.configure(bg=BG);root.geometry('1440x950+80+80');root.minsize(1100,800)
        root.grid_rowconfigure(4,weight=1);root.grid_columnconfigure(0,weight=1)
        header=tk.Frame(root,bg=BG);header.grid(row=0,column=0,sticky='ew',padx=22,pady=(16,8))
        label(header,'双 IMU · 通信监视',22).pack(side='left')
        self.mode_label=label(header,'只读监视',10,TEAL);self.mode_label.pack(side='right')
        label(root,'独立铰接小车项目  /  前后为 A10、A3 接线身份  /  IMU 尚未装车，坐标轴未经安装标定',9,MUTED).grid(row=1,column=0,sticky='w',padx=24)
        pipeline=tk.Frame(root,bg=BG);pipeline.grid(row=2,column=0,sticky='ew',padx=22,pady=12)
        self.cards=[]
        for i,title in enumerate(('01  STM32 读取 IMU','02  24L01P 无线发送','03  电脑 USB RX 接收','04  完整包交付核对')):
            pipeline.grid_columnconfigure(i,weight=1,uniform='pipeline')
            box=tk.Frame(pipeline,bg=PANEL,highlightbackground=EDGE,highlightthickness=1)
            box.grid(row=0,column=i,sticky='nsew',padx=(0,9 if i<3 else 0))
            label(box,title,10,MUTED).pack(anchor='w',padx=12,pady=(9,4))
            value=label(box,'等待连接',13,TEAL,anchor='w');value.pack(fill='x',padx=12)
            detail=label(box,'—',9,FG,anchor='w');detail.pack(fill='x',padx=12,pady=3)
            note=label(box,'—',8,MUTED,anchor='w',wraplength=300,justify='left');note.pack(fill='x',padx=12,pady=(0,9))
            self.cards.append((value,detail,note))
        control=tk.Frame(root,bg=BG);control.grid(row=3,column=0,sticky='ew',padx=22,pady=(0,8))
        label(control,'图表来源',10,MUTED).pack(side='left')
        self.source=tk.StringVar(value='历史实测回放' if offline else '无线接收')
        style=ttk.Style(root);style.theme_use('clam')
        style.configure('TCombobox',fieldbackground=PANEL,background=EDGE,foreground=FG,arrowcolor=FG)
        sources=('无线接收','STM32 旁路诊断','历史实测回放') if not offline else ('历史实测回放',)
        combo=ttk.Combobox(control,textvariable=self.source,values=sources,state='readonly',width=20,font=(FONT,10))
        combo.pack(side='left',padx=9)
        self.button(control,'▶ 回放实测记录',self.start_replay).pack(side='left',padx=4)
        self.button(control,'暂停回放',self.pause_replay).pack(side='left',padx=4)
        self.speed=tk.StringVar(value='1×')
        ttk.Combobox(control,textvariable=self.speed,values=('1×','2×','5×'),state='readonly',width=4).pack(side='left',padx=4)
        self.speed.trace_add('write',self.change_speed)
        if hardware:
            self.rx_button=self.button(control,'暂停 USB 接收',self.toggle_usb);self.rx_button.pack(side='right',padx=4)
            self.swd_button=self.button(control,'暂停 STM32 诊断',self.toggle_swd);self.swd_button.pack(side='right',padx=4)
        self.banner=tk.StringVar(value='正在读取连接状态…')
        plots=tk.Frame(root,bg=BG);plots.grid(row=4,column=0,sticky='nsew',padx=22)
        plots.grid_columnconfigure(0,weight=1,uniform='sensor');plots.grid_columnconfigure(1,weight=1,uniform='sensor')
        self.sensor_text={};self.raw_text={};self.plots={}
        groups=(('姿态角','euler_deg','°',('Roll','Pitch','Yaw')),('加速度','accel_m_s2','m/s²',('X','Y','Z')),('角速度','gyro_deg_s','°/s',('X','Y','Z')))
        for column,name in enumerate(('front','rear')):
            info=tk.Frame(plots,bg=BG);info.grid(row=0,column=column,sticky='ew',padx=(0,10) if column==0 else (0,0),pady=(0,5))
            label(info,'前 IMU · A10' if column==0 else '后 IMU · A3',13).pack(anchor='w')
            self.sensor_text[name]=label(info,'尚无当前来源的数据',9,MUTED,justify='left',anchor='w');self.sensor_text[name].pack(fill='x')
            for row,(title,key,unit,names) in enumerate(groups,1):
                plots.grid_rowconfigure(row,weight=1)
                plot=Plot(plots,title,key,unit,names);plot.grid(row=row,column=column,sticky='nsew',padx=(0,10) if column==0 else (0,0),pady=3)
                self.plots[name,key]=plot
            self.raw_text[name]=label(plots,'原始帧：—',8,MUTED,wraplength=650,justify='left',anchor='w')
            self.raw_text[name].grid(row=4,column=column,sticky='ew',pady=(4,0))
        self.banner_label=label(root,'',10,AMBER,textvariable=self.banner,anchor='w',justify='left',wraplength=1370)
        self.banner_label.grid(row=5,column=0,sticky='ew',padx=22,pady=(9,4))
        footer=tk.Frame(root,bg=BG);footer.grid(row=6,column=0,sticky='ew',padx=22,pady=(0,10))
        self.footer=label(footer,'',8,MUTED,anchor='w');self.footer.pack(side='left',fill='x',expand=True)
        self.button(footer,'打开记录文件夹',self.open_folder).pack(side='right')
        self.error=error
        root.after(100,self.update)

    def button(self,parent,text,command):
        return tk.Button(parent,text=text,command=command,bg=EDGE,fg=FG,activebackground='#35506d',activeforeground=FG,
                         relief='flat',borderwidth=0,padx=10,pady=5,font=(FONT,9),cursor='hand2')

    def toggle_usb(self):
        e=self.hardware.usb_enabled
        if e.is_set():e.clear();self.rx_button.configure(text='恢复 USB 接收')
        else:e.set();self.rx_button.configure(text='暂停 USB 接收')

    def toggle_swd(self):
        e=self.hardware.swd_enabled
        if e.is_set():e.clear();self.swd_button.configure(text='恢复 STM32 诊断')
        else:e.set();self.swd_button.configure(text='暂停 STM32 诊断')

    def open_folder(self):
        path=self.folder if self.folder.exists() else self.folder.parent
        subprocess.Popen(['xdg-open',str(path)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

    def start_replay(self):
        if not self.archive:return
        self.source.set('历史实测回放');self.play_cursor=0;self.play_start=time.monotonic();self.playing=True
        self.replay_time=self.archive[0]['host_received_monotonic_s']
        self.replay_base=self.replay_time;self.play_speed=float(self.speed.get()[0])

    def change_speed(self,*unused):
        self.replay_base=self.replay_time;self.play_start=time.monotonic();self.play_speed=float(self.speed.get()[0])

    def pause_replay(self):self.playing=False

    def card(self,index,value,detail,note,color=TEAL):
        a,b,c=self.cards[index];a.configure(text=value,fg=color);b.configure(text=detail);c.configure(text=note)

    def update(self):
        now=time.monotonic();s=self.state.snapshot(now);m=s['mcu'];radio=(m or {}).get('radio',{})
        recent=m is not None and s['mcu_seen'] is not None and now-s['mcu_seen']<3 and s['swd']['connected']
        if recent:
            rates=['—' if s['source_hz'][n] is None else f"{s['source_hz'][n]:.1f}" for n in ('front','rear')]
            self.card(0,f'前 {rates[0]} Hz  /  后 {rates[1]} Hz',
                      f"有效帧 {m['sensors']['front']['valid_frames']} / {m['sensors']['rear']['valid_frames']}",
                      f"UART 错误 {m['sensors']['front']['uart_errors']}/{m['sensors']['rear']['uart_errors']} · 队列丢失 {m['sensors']['front']['queue_drops']}/{m['sensors']['rear']['queue_drops']}")
        else:self.card(0,'STM32 诊断未连接','USB 接收可独立运行',s['swd']['error'] or '等待 ST-Link 连接',AMBER)
        if recent and radio:
            st=radio['state'];message={1:'发送窗口内',2:'SPI 初始化失败',4:'发送窗口已结束'}.get(st,'等待状态')
            if st==1 and m['mcu_uptime_ms']<radio['start_ms']:message='等待启动发送'
            self.card(1,message,f"整包 {radio['packets_completed']}  /  片段成功 {radio['fragments_ok']}",
                      f"启动 #{radio['boot_id']} · 失败片段 {radio['fragments_failed']} · SPI 错误 {radio['spi_errors']}",AMBER if st in (2,4) else TEAL)
        else:self.card(1,'暂无源端发送状态','无线计数需 ST-Link 诊断','不会根据电脑收包反推 STM32 状态',MUTED)
        c=s['rx_status']['counts']
        self.card(2,'USB RX 已连接' if s['usb']['connected'] else 'USB RX 未连接',
                  f"本会话 {c['accepted']} 包 · {s['usb']['bytes']} 字节",
                  f"CRC 错误 {s['crc_errors']} · 重复 {c['duplicates']} · 乱序 {c['out_of_order']}" if s['usb']['connected'] else (s['usb']['error'] or '接收暂停或设备未连接'),
                  TEAL if s['usb']['connected'] else AMBER)
        comparison=s['comparison']
        if 'matched' in comparison:
            self.card(3,f"{comparison['received']} / {comparison['sent']} 完整包",comparison['reason'],'已覆盖本次启动的发送全过程',TEAL if comparison['matched'] else AMBER)
        else:self.card(3,'等待窗口结束' if comparison['complete_window_observed'] else '本会话未覆盖完整窗口',
                       f"源端启动累计 {comparison['sent'] if comparison['sent'] is not None else '—'}  /  同启动实收 {comparison['received']}",comparison['reason'],MUTED)
        selected=self.source.get();hist={};latest={};plot_now=now
        if selected=='历史实测回放':
            if self.archive:
                if self.playing:
                    elapsed=(now-self.play_start)*self.play_speed;self.replay_time=self.replay_base+elapsed
                    while self.play_cursor<len(self.archive) and self.archive[self.play_cursor]['host_received_monotonic_s']<=self.replay_time:self.play_cursor+=1
                    if self.play_cursor==len(self.archive):self.playing=False;self.replay_time=self.archive[-1]['host_received_monotonic_s']
                for n in ('front','rear'):
                    records=[r for r in self.archive[:self.play_cursor] if r['sensor']==n]
                    hist[n]=[(r['host_received_monotonic_s'],r) for r in records]
                    if records:latest[n]=records[-1]
                plot_now=self.replay_time
            self.mode_label.configure(text=f'历史回放 · {self.play_cursor}/{len(self.archive)} 包',fg=PURPLE)
            self.banner.set('下方曲线为已录制的真实无线数据回放，不代表当前传输。上方链路卡片始终是当前实时状态。' if not self.offline else '离线查看：只展示已录制的真实无线数据，不连接 USB 或 ST-Link。')
            self.banner_label.configure(fg=PURPLE)
        else:
            src='radio' if selected=='无线接收' else 'mcu';hist=s['history'][src];latest=s['radio_latest'] if src=='radio' else s['source_latest']
            self.mode_label.configure(text='实时 · '+selected,fg=TEAL if src=='radio' else AMBER)
            self.banner_label.configure(fg=AMBER)
            if recent and radio.get('state')==4:
                self.banner.set('当前验证固件的无线窗口已结束，STM32 仍在采集。下次上电后约 5 秒开始约一分钟发送；本界面不复位、不烧录。')
            elif src=='mcu':self.banner.set('下方是 ST-Link 抽样查看的源端数据，不是空中收包或完整 25Hz 序列；完整链路请同时查看 USB RX 卡片。')
            else:self.banner.set('下方只显示 USB RX 真正收到并通过 CRC 的测量。序号缺口包含主动降频省略的源帧，不能直接当作射频丢包。')
        for name in ('front','rear'):
            row=latest.get(name);points=hist.get(name,[])
            if row:
                age=max(0,plot_now-row['host_received_monotonic_s'])
                values='  '.join(f'{x:+.2f}' for x in row['euler_deg'])
                if selected=='历史实测回放':
                    span=points[-1][0]-points[0][0] if len(points)>1 else 0
                    frequency=(len(points)-1)/span if span else None;frequency_name='记录接收'
                elif selected=='无线接收':frequency=s['radio_hz'][name];frequency_name='当前接收'
                else:frequency=s['source_hz'][name];frequency_name='源端采集'
                rate='—' if frequency is None else f'{frequency:.2f} Hz'
                self.sensor_text[name].configure(text=f"Roll / Pitch / Yaw  {values} °\n启动 {row.get('boot_id','—')} · 源序号 {row['sequence']} · 最近更新 {age:.1f}s 前 · {frequency_name} {rate}",fg=FG if age<2.5 else MUTED)
                prefix='IMU 原始帧' if selected=='STM32 旁路诊断' else '无线遥测包'
                self.raw_text[name].configure(text=f"{prefix}  {row.get('wire_hex','—')}\nMCU UART 收帧计时 {row['mcu_uart_rx_time_us']} µs；不等同于电脑时钟。")
            else:
                self.sensor_text[name].configure(text='尚无当前来源的数据；不会用零值填充缺失测量。',fg=MUTED)
                self.raw_text[name].configure(text='原始帧：—')
            for key in ('euler_deg','accel_m_s2','gyro_deg_s'):self.plots[name,key].update_data(points,plot_now)
        self.footer.configure(text=('离线 · '+(self.error or '记录已校验加载')) if self.offline else f"自动记录：{self.folder.name}  |  ST-Link 不一致快照跳过 {s['swd']['busy_reads']} 次  |  Yaw 是传感器输出，不是绝对航向")
        self.frame+=1
        if self.hardware and self.frame%8==0:
            report={k:s[k] for k in ('usb','swd','mcu','source_hz','radio_hz','comparison','rx_status','crc_errors','discarded_bytes')}
            report['host_monotonic_s']=now
            try:
                temp=self.folder/'status.json.tmp';temp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
                temp.replace(self.folder/'status.json')
            except OSError as error:self.footer.configure(text='记录状态写入失败：'+str(error),fg=AMBER)
        self.root.after(250,self.update)
