"use client"
import React, { useState, useEffect, useRef } from 'react';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';
import Fab from '@mui/material/Fab';
import { PanelContainer, MasconHandle, ReverserHandle, SemicircleMascon, MechanicalSwitch } from './style';
import { Typography, IconButton, Tooltip } from '@mui/material';
import FullscreenButton from '@/components/FullScreenButton';

export default function ControllerApp() {
    const [masconValue, setMasconValue] = useState(0);
    const [reverserValue, setReverserValue] = useState(0);
    const [angleValue, setAngleValue] = useState(0);
    const [manualMode, setManualMode] = useState(false);
    const [rotateValue, setRotateMode] = useState(false);
    const [isPortrait, setIsPortrait] = useState(false);
    const ws = useRef<WebSocket | null>(null);

    const handleChange = (event: any) => {
        setManualMode(event.target.checked);
    };
    const rotateModeChange = (event: any) => {
        setRotateMode(event.target.checked);
    };
    const [scale, setScale] = useState(1);

    useEffect(() => {
        ws.current = new WebSocket("/api/local/ws");
        
        ws.current.onopen = () => {
            console.log("WebSocket Connected");
        };

        const handleResize = () => {
            const h = window.innerHeight;
            const w = window.innerWidth;
            
            setIsPortrait(h > w);

            const threshold = 450;
            const newScale = h <= threshold ? h / threshold : 1;
            setScale(newScale);
        };

        handleResize();
        window.addEventListener('resize', handleResize);
        return () => {
            window.removeEventListener('resize', handleResize);
            if(ws.current){
                ws.current.close();
            }
        };
    }, []);

    useEffect(() => {
        const sendControlData = () => {
            if (ws.current && ws.current.readyState === WebSocket.OPEN && manualMode) {
                const data = {
                    type: "control",
                    direction: reverserValue,
                    speed: masconValue * -1,
                    angle: angleValue,
                    rotate: rotateValue
                };
                ws.current.send(JSON.stringify(data));
            }
        };
        sendControlData();
    }, [masconValue, reverserValue, angleValue, rotateValue, manualMode]);

    useEffect(() => {
        const set_manual_mode = (mode: boolean) => {
            if (ws.current && ws.current.readyState === WebSocket.OPEN) {
                ws.current.send(JSON.stringify({
                    type: "manual_mode",
                    mode: mode
                }));
            }
        };
        set_manual_mode(manualMode);
        setMasconValue(0);
        setReverserValue(0);
        setAngleValue(0);
        setRotateMode(false);
    }, [manualMode]);

    const handleMasconChange = (event: any, newValue: number | number[]) => {
        setMasconValue(Array.isArray(newValue) ? newValue[0] : newValue);
    };

    const handleReverserChange = (event: any, newValue: number | number[]) => {
        setReverserValue(Array.isArray(newValue) ? newValue[0] : newValue);
    };
    const upLinear = () => {
        if (manualMode && ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify({
                type: "linear",
                mode: "up"
            }));
        }
    };

    const downLinear = () => {
        if (manualMode && ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify({
                type: "linear",
                mode: "down"
            }));
        }
    };

    const stopLinear = () => {
        if (manualMode && ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify({
                type: "linear",
                mode: "stop"
            }));
        }
    };

    const openSlide = () => {
        if (manualMode && ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify({
                type: "slide",
                mode: "open"
            }));
        }
    }

    const closeSlide = () => {
        if (manualMode && ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify({
                type: "slide",
                mode: "close"
            }));
        }
    }

    const handleBeforeUnload = () => {
        setManualMode(false);
        if (ws.current && ws.current.readyState === WebSocket.OPEN) {
            ws.current.send(JSON.stringify({
                type: "manual_mode",
                mode: false
            }));
            ws.current.send(JSON.stringify({
                type: "control",
                direction: 0,
                speed: 0,
                angle: 0,
                rotate: false
            }));
        }
    }

    useEffect(() => {
        window.addEventListener('beforeunload', handleBeforeUnload)

        return () => {
            window.removeEventListener('beforeunload', handleBeforeUnload)
        }
    }, [handleBeforeUnload])

    if (isPortrait) {
        return (
            <Box sx={{ 
                position: "absolute", 
                top: 0, 
                left: 0, 
                width: "100vw", 
                height: "100vh", 
                backgroundColor: '#222222',
                display: 'flex',
                justifyContent: 'center',
                alignItems: 'center',
                zIndex: 9999
            }}>
                <Typography variant="h5" color="white" fontWeight="bold">
                    横画面にしてください
                </Typography>
            </Box>
        );
    }

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
                    <Grid width="calc(100%/3*2 - 10%)" display={"flex"} height={"100%"} sx={{ textAlign: "left" }} >
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
                        <Box sx={{ display: "flex", flexDirection: "column", ml:2}}>
                            <PanelContainer sx={{ display: "flex", justifyContent: "space-around", flexDirection: "column", width: "auto" }}>
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
                                    onMouseDown={upLinear}
                                    onMouseUp={stopLinear}
                                    onMouseLeave={stopLinear}
                                    onTouchStart={upLinear}
                                    onTouchEnd={stopLinear}
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
                                    onMouseDown={downLinear}
                                    onMouseUp={stopLinear}
                                    onMouseLeave={stopLinear}
                                    onTouchStart={downLinear}
                                    onTouchEnd={stopLinear}
                                >
                                    下
                                </Fab>
                            </PanelContainer>
                            <PanelContainer sx={{ display: "flex", justifyContent: "space-around", flexDirection: "column", width: "auto"}}>
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
                                    onClick={openSlide}
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
                                    onClick={closeSlide}
                                >
                                    収
                                </Fab>
                            </PanelContainer>
                        </Box>
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
                        <PanelContainer sx={{ pt: 0, pb: 2, pl: 2, pr: 2 }}>
                            <SemicircleMascon
                                value={angleValue}
                                onChange={setAngleValue}
                            />
                            <Box sx={{ display: "flex", flexDirection: "column" }}>
                                <Typography sx={{ color: "white", marginBottom: 1, width: "100%", textAlign: "center" }}>超信地旋回</Typography>
                                <Box display={"flex"}>
                                    <Typography sx={{ color: "white", m: 0.5 }}>切</Typography>
                                    <MechanicalSwitch
                                        checked={rotateValue}
                                        onChange={rotateModeChange}
                                        inputProps={{ 'aria-label': 'mechanical switch' }}
                                        disableRipple
                                    />
                                    <Typography sx={{ color: "red", m: 0.5 }}>入</Typography>
                                </Box>
                            </Box>
                        </PanelContainer>
                    </Grid>
                </Box>
            </Box>
            <FullscreenButton />
        </Box>
    );
}