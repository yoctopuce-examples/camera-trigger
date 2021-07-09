/*********************************************************************
 *
 *  OMRON V430-F camera driver based on RangeFinder detection
 *
 *  Camera settings:
 *  - Cycle Triggered by <SP>, timeout after 2000ms
 *  - Capture mode Continuous, delay 0us
 *  - Look for 1 symbol
 *  Acquire: flash 29000us, 33%, 264mm, enhance disabled
 *  Decode: Data Matrix
 *  Autofocus
 *
 * URL pour charger l'image: http://172.17.17.50/api/v1/image?decimate=1
 *
 *********************************************************************/

import { YAPI, YErrorMsg, YModule, YFunction, YMeasure } from 'yoctolib-cjs/yocto_api_nodejs.js';
import { YRangeFinder } from 'yoctolib-cjs/yocto_rangefinder.js';
import { YBuzzer } from 'yoctolib-cjs/yocto_buzzer.js';
import { Socket } from 'net';
import * as http from 'http';
import * as fs from 'fs';

const RANGEFINDER_IP: string = '172.17.17.101';
const BUZZER_IP: string = '172.17.17.61';
const CAMERA_IP: string = '172.17.17.50';

const THRESHOLD_DISTANCE_MM: number = 50;
const THRESHOLD_MAX_DEVIATION: number = 1;

let buzzer: YBuzzer | null = null;
let lastMeasures: number[] = [];
let camera: V430F_Driver;

class V430F_Driver
{
    _ipaddr: string;
    _port: number;
    _sock: Socket | null;
    _captureTimeout: NodeJS.Timeout | null;
    _targetPattern: boolean | null;

    constructor(ipaddr: string, port: number)
    {
        this._ipaddr = ipaddr;
        this._port = port;
        this._sock = null;
        this._captureTimeout = null;
        this._targetPattern = null;
    }

    public connect(): Promise<void>
    {
        return new Promise<void>(
            (resolve, reject) => {
                let sock = new Socket();
                sock.on('data', (data: Buffer) => {
                    this.captureResult(data);
                });
                sock.on('close', () => {
                    console.log('Connection to V430-F closed');
                    this._sock = null;
                });
                sock.connect(this._port, this._ipaddr, () => {
                    console.log('Connected to V430-F on ' + this._ipaddr + ':' + this._port);
                    this._sock = sock;
                    resolve();
                });
            }
        );
    }

    public async setTargetPattern(enabled: boolean): Promise<void>
    {
        if(this._targetPattern == enabled) return;
        if(this._captureTimeout) return;
        if(this._sock === null) {
            await this.connect();
        }
        if(this._sock === null) {
            return;
        }
        this._targetPattern = enabled;
        this._sock.write('<l'+(enabled?1:0)+'>\r\n');
    }

    public async downloadImage(decimateFactor: number): Promise<Buffer>
    {
        return new Promise<Buffer>((resolve, reject) => {
            let url = 'http://' + this._ipaddr + '/api/v1/image?decimate='+decimateFactor;
            http.get(url, (res: http.IncomingMessage) => {
                if (res.statusCode != 200 && res.statusCode != 304) {
                    if (res.statusCode) {
                        reject(new Error('HTTP error ' + res.statusCode));
                    } else {
                        reject(new Error('Unable to complete HTTP request'))
                    }
                } else {
                    let response = Buffer.alloc(0);
                    res.on('data', (chunk: Buffer) => {
                        response = Buffer.concat([response, chunk]);
                    });
                    res.on('end', () => {
                        resolve(response);
                    })
                }
            }).on('error', (e: Error) => {
                reject(new Error('HTTP error: ' + e.message));
            });
        });
    }

    public async triggerCapture(): Promise<void>
    {
        // If a capture is already pending, exit immediately
        if(this._captureTimeout) return;
        if(this._sock === null) {
            await this.connect();
        }
        if(this._sock === null) {
            return;
        }
        this._sock.write('< >\r\n');
        this._captureTimeout = setTimeout(() => {
            // retry after timeout if no result received
            this._captureTimeout = null;
            this.triggerCapture();
        }, 5000);
    }

    protected async captureResult(data: Buffer): Promise<void>
    {
        let result = data.toString().trim();
        if(this._captureTimeout) {
            clearTimeout(this._captureTimeout);
        }
        if(result == '' || result.indexOf('NOREAD') >= 0) {
            console.log('No datamatrix code found');
            this._captureTimeout = null;
            if(buzzer) buzzer.playNotes("C48 C C ,C#8");
            return;
        }
        let now: Date = new Date();
        let day: string = now.getFullYear().toString() + '-' + ('0' + (now.getMonth() + 1)).slice(-2) + '-' + ('0' + now.getDate()).slice(-2);
        let time: string = now.getHours().toString() + 'h' + ('0' + now.getMinutes()).slice(-2) + 'm' + ('0' + now.getSeconds()).slice(-2);
        let stamp: string = day + '_' + time;
        let fields: any = {};
        let separator: number = 0x1d;
        let endpos: number = data.length;
        let prevpos: number = 0;
        while(endpos > 0 && [4,13,10,30].includes(data[endpos-1])) {
            endpos--;
        }
        do {
            let pos: number = data.indexOf(separator, prevpos);
            if(pos < 0) pos = endpos;
            let field: string = data.toString('utf-8', prevpos, pos).trim().replace(/\u001e/g,'<RS>');
            if(field.slice(0,4) == 'GWCR') {
                // TT Electronics proprietary encoding
                let parts: string[] = field.split('|');
                fields['mfg_pn'] = parts[0];
                fields['qty'] = parts[1];
                fields['date_code'] = parts[2];
                fields['coo'] = parts[3].slice(5, 7);
                fields['traceability'] = parts[3].slice(7);
            } else if(field.indexOf('[)>') >= 0) {
                // Typical ANSI/MH10.8 signature
                fields['signature'] = field;
            } else if(field.slice(0,2) == '1P') {
                fields['mfg_pn'] = field.slice(2);
            } else if(field.slice(0,2) == '1V') {
                fields['manufacturer'] = field.slice(2);
            } else if(field.slice(0,2) == '1T') {
                fields['traceability'] = field.slice(2);
            } else if(field.slice(0,2) == '4L') {
                fields['coo'] = field.slice(2);
            } else if(field[0] == 'P') {
                fields['product'] = field.slice(1);
            } else if(field[0] == 'Q') {
                fields['qty'] = field.slice(1);
            } else if(field[0] == 'S') {
                fields['serial'] = field.slice(1);
            } else if(field.slice(0,2).match(/[4-9]D/)) {
                fields['date_code'] = field.slice(2);
            } else if(field.slice(0,3).match(/1[0-6]D/)) {
                fields['date_code'] = field.slice(3);
            } else if(field[0] == 'K') {
                if(field.length == 1) {
                    // Empty customer order number
                    fields['distributor'] = 'Digi-Key';
                } else {
                    fields['order_no'] = field.slice(1);
                }
            } else if(fields['distributor'] == 'Digi-Key') {
                if (field.slice(0, 2) == '1K') {
                    fields['order_no'] = field.slice(2);
                } else if (field.slice(0, 3) == '4K') { // Farnell
                    fields['line_no'] = field.slice(2);
                } else if (field.slice(0, 3) == '10K') {
                    fields['invoice_no'] = field.slice(3);
                } else if (field.slice(0, 3) == '11K') { // Mouser
                    fields['line_no'] = field.slice(3);
                } else if (field.slice(0, 3) == '11Z') {
                    fields['pick'] = field.slice(3);
                } else if (field.slice(0, 3) == '12Z') {
                    fields['part_id'] = field.slice(3);
                } else if (field.slice(0, 3) == '13Z') {
                    fields['load_id'] = field.slice(3);
                } else if (field.slice(0, 3) == '20Z') {
                    // ignore (zero padding)
                }
            } else if(field.slice(0,3) == '11K') {
                // Mouser
                fields['invoice_no'] = field.slice(3);
            } else if(field.slice(0,3) == '14K') {
                // Mouser
                fields['line_no'] = field.slice(3);
            } else {
                fields['unknown'] = field;
            }
            prevpos = pos+1;
        } while(prevpos < endpos);
        if(fields['mfg_pn']) {
            console.log(stamp + ': Datamatrix detected ('+fields['mfg_pn']+')');
            http.get('http://172.17.17.77/FR/admin/feederScannerCallback.php?search='+fields['mfg_pn']);
        } else {
            // no decoding available for this code
            if(buzzer) buzzer.playNotes("'F48 A C G ,C G C G C");
        }
        this.setTargetPattern(false);
        fs.writeFileSync(stamp + '_data.bin', data);
        fs.writeFileSync(stamp + '_data.json', JSON.stringify(fields, null, 1));
        let img = await this.downloadImage(1);
        fs.writeFileSync(stamp + '_image.jpg', img);
        console.log(stamp + ': image saved');
        this._captureTimeout = null;
    }
}

let prevLog: number = Date.now();

async function timedReportCallback(obj_fct: YFunction, value: YMeasure): Promise<void>
{
    // Keep last 5 measures
    lastMeasures.push(value.get_averageValue());
    let n: number = lastMeasures.length;
    if(n < 5) return;
    let sum: number = 0;
    let sumsq: number = 0;
    for(let i = 0; i < n; i++) {
        sum += lastMeasures[i];
        sumsq += lastMeasures[i] * lastMeasures[i];
    }
    let avg = sum / n;
    let stddev = Math.sqrt(sumsq / n - avg*avg);
    if(Date.now() - prevLog > 3000) {
        console.log('Average: ' + avg + ', std dev: ' + Math.round(stddev*100)/100 + ': ' + JSON.stringify(lastMeasures));
        prevLog = Date.now();
    }
    lastMeasures.splice(0, 1);
    if(avg < THRESHOLD_DISTANCE_MM && stddev < THRESHOLD_MAX_DEVIATION) {
        camera.triggerCapture();
    } else if(avg < THRESHOLD_DISTANCE_MM || stddev > THRESHOLD_MAX_DEVIATION) {
        camera.setTargetPattern(true);
    } else if(avg > THRESHOLD_DISTANCE_MM && stddev < THRESHOLD_MAX_DEVIATION) {
        camera.setTargetPattern(false);
    }
}

async function deviceArrival(module: YModule): Promise<void>
{
    let serial: string = await module.get_serialNumber();
    let lname: string = await module.get_logicalName();
    console.log('Device arrival: '+serial+' (logical name: '+lname+')');
    if(lname == 'CameraRangeFinder') {
        let rangeFinder = YRangeFinder.FindRangeFinder('CameraRangeFinder');
        rangeFinder.set_timeFrame(0);
        rangeFinder.set_rangeFinderMode(YRangeFinder.RANGEFINDERMODE.HIGH_ACCURACY);
        rangeFinder.set_reportFrequency('10/s');
        rangeFinder.registerTimedReportCallback(timedReportCallback);
    }
    if(lname == 'FeederBuzzer') {
        buzzer = YBuzzer.FindBuzzer('FeederBuzzer');
    }
}

async function deviceRemoval(module: YModule): Promise<void>
{
    let serial: string = await module.get_serialNumber();
    console.log('Device removal : '+serial);
}

function handleHotPlug()
{
    let errmsg = new YErrorMsg();
    YAPI.UpdateDeviceList(errmsg);
    setTimeout(handleHotPlug, 2000);
}

async function startDemo(): Promise<void>
{
    await YAPI.LogUnhandledPromiseRejections();
    await YAPI.DisableExceptions();
    await YAPI.RegisterDeviceArrivalCallback(deviceArrival);
    await YAPI.RegisterDeviceRemovalCallback(deviceRemoval);

    // Setup the API to use the VirtualHub on local machine
    let errmsg = new YErrorMsg();
    if (await YAPI.PreregisterHub(RANGEFINDER_IP, errmsg) !== YAPI.SUCCESS) {
        console.log('Cannot contact YoctoHub-Ethernet on 172.17.17.101: ' + errmsg.msg);
        return;
    }
    if (await YAPI.PreregisterHub(BUZZER_IP, errmsg) !== YAPI.SUCCESS) {
        console.log('Cannot contact YoctoHub-Ethernet on 172.17.17.61: ' + errmsg.msg);
        return;
    }
    handleHotPlug()
    camera = new V430F_Driver(CAMERA_IP, 2001);
    camera.connect();
}

startDemo();
