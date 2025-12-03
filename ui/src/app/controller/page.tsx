"use client"
import React, { useState, useEffect, useRef } from 'react';
import Slider from '@mui/material/Slider';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';
import FormGroup from '@mui/material/FormGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import FormControl from '@mui/material/FormControl';
import Switch from '@mui/material/Switch';
import { PanelContainer, MasconHandle, ReverserHandle, SemicircleMascon, MechanicalSwitch } from './style';
import { Typography } from '@mui/material';

const API_HOST = "http://localhost:8100";

export default function ControllerApp() {
    const [masconValue, setMasconValue] = useState(0);
    const [reverserValue, setReverserValue] = useState(0);
    const [angleValue, setAngleValue] = useState(0);
    const [manualMode, setManualMode] = useState(false);
    const handleChange = (event: any) => {
        setManualMode(event.target.checked);
    };
    const [scale, setScale] = useState(1);

    useEffect(() => {
        const handleResize = () => {
            const h = window.innerHeight;
            const threshold = 450;
            const newScale = h <= threshold ? h / threshold : 1;
            setScale(newScale);
        };

        handleResize();
        window.addEventListener('resize', handleResize);
        return () => window.removeEventListener('resize', handleResize);
    }, []);

    /* 操作データ送信 */
    useEffect(() => {
        const sendControlData = async () => {
            try {
                await fetch(API_HOST + `/controll_api?direction=${reverserValue}&speed=${masconValue * -1}&angle=${angleValue}`, {
                    method: 'GET'
                });
            } catch (error) {
                console.error('Failed to send control data:', error);
            }
        };
        sendControlData();
    }, [masconValue, reverserValue, angleValue]);

    /* 手動制御モード設定 */
    useEffect(() => {
        const set_manual_mode = async (mode: boolean) => {
            try {
                await fetch(API_HOST + `/set_manual_mode?mode=${mode ? true : false}`, {
                    method: 'GET'
                });
            } catch (error) {
                console.error('Failed to set manual mode:', error);
            }
        };
        set_manual_mode(manualMode);
        setMasconValue(0);
        setReverserValue(0);
        setAngleValue(0);
    }, [manualMode]);

    const handleMasconChange = (event: any, newValue: number | number[]) => {
        setMasconValue(Array.isArray(newValue) ? newValue[0] : newValue);
    };

    const handleReverserChange = (event: any, newValue: number | number[]) => {
        setReverserValue(Array.isArray(newValue) ? newValue[0] : newValue);
    };

    /*操作画面を閉じるときは停止し制御を切る*/
    const handleBeforeUnload = async() => {
        setManualMode(false);
        try {
            await fetch(API_HOST + `/set_manual_mode?mode=false`, {
                method: 'GET'
            });
        } catch (error) {
            console.error('Failed to set manual mode:', error);
        }

        try {
            await fetch(API_HOST + `/controll_api?direction=0&speed=0&angle=0`, {
                    method: 'GET'
            });
        } catch (error) {
            console.error('Failed to send control data:', error);
        }
    }

    useEffect(() => {
        window.addEventListener('beforeunload', handleBeforeUnload)

        return () => {
            window.removeEventListener('beforeunload', handleBeforeUnload)
        }
    }, [handleBeforeUnload])

    return (
        <Box sx={{ position: "absolute", top: 0, left: 0, width: "100vw", height: "100vh", backgroundColor: '#222222' }}>
            <Box sx={{
                display: 'flex', flexDirection: 'column', width: "100%",
                transform: `scale(${scale})`,
                transformOrigin: 'top center',
                transition: 'transform 0.1s linear'
            }}>
                <Box
                    sx={{
                        display: 'flex',
                        flexDirection: 'row',
                        alignItems: 'center',
                        justifyContent: 'center',
                        userSelect: 'none',
                    }}
                >
                    <Grid width="calc(100%/3)" display={"flex"} height={"100%"}>
                        <PanelContainer sx={{ mr: 2, marginLeft: 5 }}>
                            <ReverserHandle
                                value={reverserValue}
                                onChange={handleReverserChange}
                                sx={{ height: 180 }}
                            />
                        </PanelContainer>
                        <PanelContainer sx={{ flexShrink: 3, height: "100%", scale: 1.0 }}>
                            <MasconHandle
                                value={masconValue}
                                onChange={handleMasconChange}
                                sx={{ marginTop: 2 }}
                            />
                        </PanelContainer>
                    </Grid>

                    <Grid
                        width="calc(100%/3 - 10%)"
                    >
                        {/* 空白 ボタン等入れる予定 */}
                    </Grid>

                    <Grid width="calc(100%/3 + 10%)" display={"flex"} flexDirection={"column"}>
                        <PanelContainer sx={{ display: "flex", flexDirection: "row" }}>
                            <Box sx={{ display: "flex", flexDirection: "column" }}>
                                <Typography sx={{ color: "white", marginBottom: 1, width: "100%", textAlign: "center" }}>手動制御</Typography>
                                <Box display={"flex"}>
                                    <Typography sx={{ color: "white", m: 0.5 }}>切</Typography>
                                    <MechanicalSwitch
                                        checked={manualMode}
                                        onChange={handleChange}
                                        inputProps={{ 'aria-label': 'mechanical switch' }}
                                        disableRipple
                                    />
                                    <Typography sx={{ color: "red", m: 0.5 }}>入</Typography>
                                </Box>
                            </Box>
                        </PanelContainer>
                        <PanelContainer sx={{ p: 2 }}>
                            <SemicircleMascon
                                value={angleValue}
                                onChange={setAngleValue}
                            />
                        </PanelContainer>
                    </Grid>
                </Box>
            </Box>
        </Box>
    );
}