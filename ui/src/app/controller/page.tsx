"use client"
import React, { useState, useEffect, useRef } from 'react';
import Slider from '@mui/material/Slider';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';
import Fab from '@mui/material/Fab';
import FormGroup from '@mui/material/FormGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import FormControl from '@mui/material/FormControl';
import Switch from '@mui/material/Switch';
import { PanelContainer, MasconHandle, ReverserHandle, SemicircleMascon, MechanicalSwitch } from './style';
import { Typography } from '@mui/material';

const API_HOST = "/api/local";

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

    const timerRef = useRef<NodeJS.Timeout | undefined>(undefined);

    const upLinear = async () => {
        try {
            await fetch(API_HOST + `/linear?mode=up`, {
                method: 'GET'
            });
        } catch (error) {
            console.error('Failed to set manual mode:', error);
        }
    };

    const downLinear = async () => {
        try {
            await fetch(API_HOST + `/linear?mode=down`, {
                method: 'GET'
            });
        } catch (error) {
            console.error('Failed to set manual mode:', error);
        }
    };

    const stopLinear = async () => {
        try {
            await fetch(API_HOST + `/linear?mode=stop`, {
                method: 'GET'
            });
        } catch (error) {
            console.error('Failed to set manual mode:', error);
        }
    };

    const up_startPress = () => {
        upLinear();
    };

    const up_stopPress = () => {
        stopLinear();
    };

    const down_startPress = () => {
        downLinear();
    };

    const down_stopPress = () => {
        stopLinear();
    };

    /*操作画面を閉じるときは停止し制御を切る*/
    const handleBeforeUnload = async () => {
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
                    <Grid width="calc(100%/3*2 - 10%)" display={"flex"} height={"100%"} sx={{textAlign: "left"}} >
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
                        <Box sx={{display:"flex", flexDirection:"column"}}>
                        <PanelContainer sx={{ display: "flex", justifyContent: "space-around", flexDirection: "column", width: "25%" }}>
                            <Fab
                                aria-label="add"
                                sx={{
                                    backgroundColor: 'white',
                                    color: 'Black',
                                    '&:hover': {
                                        backgroundColor: '#f5f5f5',
                                    },
                                    marginBottom: 1,
                                    fontSize: '1.2rem'
                                }}
                                onMouseDown={up_startPress}  
                                onMouseUp={up_stopPress}     
                                onMouseLeave={up_stopPress}  
                                onTouchStart={up_startPress} 
                                onTouchEnd={up_stopPress}   
                            >
                                上
                            </Fab>
                            <Fab
                                aria-label="add"
                                sx={{
                                    backgroundColor: '#B40000',
                                    color: 'White',
                                    '&:hover': {
                                        backgroundColor: '#B40000',
                                    },
                                    marginTop: 1,
                                    fontSize: '1.2rem'
                                }}
                                onMouseDown={down_startPress}  
                                onMouseUp={down_stopPress}     
                                onMouseLeave={down_stopPress}  
                                onTouchStart={down_startPress} 
                                onTouchEnd={down_stopPress}   
                            >
                                下
                            </Fab>
                        </PanelContainer>
                        <PanelContainer sx={{ display: "flex", justifyContent: "space-around", flexDirection: "column", width: "25%" }}>
                            <Fab
                                aria-label="add"
                                sx={{
                                    backgroundColor: 'white',
                                    color: 'Black',
                                    '&:hover': {
                                        backgroundColor: '#f5f5f5',
                                    },
                                    marginBottom: 1,
                                    fontSize: '1.2rem'
                                }}
                                onMouseDown={up_startPress}  
                                onMouseUp={up_stopPress}     
                                onMouseLeave={up_stopPress}  
                                onTouchStart={up_startPress} 
                                onTouchEnd={up_stopPress}   
                            >
                                展
                            </Fab>
                            <Fab
                                aria-label="add"
                                sx={{
                                    backgroundColor: '#ffffffff',
                                    color: 'Black',
                                    '&:hover': {
                                        backgroundColor: '#B40000',
                                    },
                                    marginTop: 1,
                                    fontSize: '1.2rem'
                                }}
                                onMouseDown={down_startPress}  
                                onMouseUp={down_stopPress}     
                                onMouseLeave={down_stopPress}  
                                onTouchStart={down_startPress} 
                                onTouchEnd={down_stopPress}   
                            >
                                収
                            </Fab>
                        </PanelContainer>
                        </Box>
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