"use client"
import React, { useState, useEffect, useRef } from 'react';
import { 
    Box, 
    Typography, 
    Card, 
    Button, 
    Dialog, 
    DialogTitle, 
    DialogContent, 
    DialogContentText, 
    DialogActions 
} from "@mui/material";
import Map from '@/components/Map'

interface Point {
    name: string,
    x: number;
    y: number;
    angle: number;
}

export default function Page() {
    const [points, setPoints] = useState<Point[]>([]);
    const [selectedPointIndex, setSelectedPointIndex] = useState<number | null>(null);
    
    const [isMoveConfirmOpen, setMoveConfirmOpen] = useState(false);
    const [pendingMovePoint, setPendingMovePoint] = useState<Point | null>(null);

    const containerRef = useRef<HTMLDivElement>(null);
    const contentRef = useRef<HTMLDivElement>(null);
    const [scale, setScale] = useState(1);

    useEffect(() => {
        const fetchPoints = async () => {
            try {
                const response = await fetch('/api/local/saved_target_points');
                const data = await response.json();
                setPoints(data);
            } catch (error) {
                console.error('Error fetching points:', error);
            }
        }
        fetchPoints();
    }, []);

    useEffect(() => {
        const container = containerRef.current;
        const content = contentRef.current;
        if (!container || !content) return;

        const updateScale = () => {
            const containerRect = container.getBoundingClientRect();
            
            const contentWidth = content.offsetWidth; 
            const contentHeight = content.offsetHeight;
            const containerWidth = containerRect.width;
            const containerHeight = containerRect.height;

            if (contentWidth === 0 || contentHeight === 0) return;

            const scaleX = containerWidth / contentWidth;
            const scaleY = containerHeight / contentHeight;
            
            const newScale = Math.min(scaleX, scaleY);
            
            setScale(newScale < 1 ? newScale : 1);
        };

        const observer = new ResizeObserver(() => {
            updateScale();
        });

        observer.observe(container);
        observer.observe(content);
        updateScale();

        return () => observer.disconnect();
    }, []);

    const handleMoveClick = (point: Point) => {
        setPendingMovePoint(point);
        setMoveConfirmOpen(true);
    };

    const executeAutoMove = async () => {
        if (!pendingMovePoint) return;
        
        try {
            const resSet = await fetch('/api/local/set_target_point', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(pendingMovePoint),
            });
            
            if (!resSet.ok) {
                console.error('Failed to set target point');
                return;
            }

            const resStart = await fetch('/api/local/automove_start');
            if (resStart.ok) {
                console.log(`Auto move started to ${pendingMovePoint.name}`);
            } else {
                console.error('Failed to start automove');
            }

        } catch (error) {
            console.error('Error sending move command:', error);
        } finally {
            setMoveConfirmOpen(false);
            setPendingMovePoint(null);
        }
    };

    return (
        <Box sx={{ 
            position: "fixed",
            inset: 0,
            width: "100%",
            height: "100%",
            overflow: "hidden",
            
            display: "flex", 
            flexDirection: "column", 
            bgcolor: "background.default",
            p: 2, 
            boxSizing: "border-box"
        }}>
            <Box sx={{ pb: 2, flexShrink: 0 }}>
                <Card sx={{ p: 1 }}>
                    <Typography>自動移動</Typography>
                </Card>
            </Box>

            <Box sx={{ 
                display: "flex", 
                flex: 1, 
                overflow: "hidden", 
                flexDirection: { xs: "column", md: "row" }, 
                gap: 2 
            }}>
                
                {/* マップ表示エリア */}
                <Box 
                    ref={containerRef}
                    sx={{ 
                        flex: 1, // 余ったスペースをすべて使う
                        display: "flex", 
                        justifyContent: "center", 
                        alignItems: "center",
                        overflow: "hidden", 
                        border: "1px solid #ddd", 
                        borderRadius: 1,
                        bgcolor: "#fff",
                        position: "relative",
                        minHeight: 0 // Flexアイテムが潰れるのを防ぐおまじない
                    }}
                >
                    <Box 
                        ref={contentRef}
                        sx={{
                            transform: `scale(${scale})`,
                            transformOrigin: "center center",
                            width: "max-content", 
                            height: "max-content",
                            display: "flex"
                        }}
                    >
                        <Map />
                    </Box>
                </Box>

                {/* 地点一覧エリア */}
                <Box sx={{ 
                    // 【修正箇所】
                    // 縦画面(xs): 幅100%, 高さ40% (残りの60%をMapが使う)
                    // 横画面(md): 幅300px, 高さ100% (残りの幅をMapが使う)
                    width: { xs: "100%", md: "300px" }, 
                    height: { xs: "40%", md: "100%" }, 
                    
                    display: "flex",
                    flexDirection: "column",
                    overflow: "hidden"
                }}>
                    <Card sx={{ p: 1, mb: 1, flexShrink: 0 }}>
                        <Typography fontWeight="bold">地点一覧</Typography>
                    </Card>
                    
                    <Box sx={{ 
                        flex: 1, 
                        overflowY: "auto", 
                        pr: 1 
                    }}>
                        {
                            points.map((point, index) => {
                                const isSelected = selectedPointIndex === index;
                                return (
                                    <Box 
                                        key={index} 
                                        onClick={() => setSelectedPointIndex(isSelected ? null : index)}
                                        sx={{ 
                                            border: isSelected ? "2px solid #1976d2" : "1px solid #ccc", 
                                            p: 1, 
                                            mb: 1, 
                                            borderRadius: 1, 
                                            bgcolor: isSelected ? "#e3f2fd" : "background.paper",
                                            cursor: "pointer",
                                            transition: "all 0.2s"
                                        }}
                                    >
                                        <Typography variant="subtitle2" fontWeight="bold">{point.name}</Typography>
                                        <Box sx={{ pl: 1, fontSize: "0.875rem" }}>
                                            <Typography variant="caption" display="block">X: {point.x}</Typography>
                                            <Typography variant="caption" display="block">Y: {point.y}</Typography>
                                            <Typography variant="caption" display="block">角度: {point.angle}</Typography>
                                        </Box>

                                        {isSelected && (
                                            <Button 
                                                variant="contained" 
                                                color="secondary" 
                                                fullWidth 
                                                size="small"
                                                sx={{ mt: 1 }}
                                                onClick={(e) => {
                                                    e.stopPropagation();
                                                    handleMoveClick(point);
                                                }}
                                            >
                                                ここに移動
                                            </Button>
                                        )}
                                    </Box>
                                );
                            })
                        }
                    </Box>
                </Box>
            </Box>

            <Dialog open={isMoveConfirmOpen} onClose={() => setMoveConfirmOpen(false)}>
                <DialogTitle>移動確認</DialogTitle>
                <DialogContent>
                    <DialogContentText>
                        地点「{pendingMovePoint?.name}」へ移動を開始しますか？
                    </DialogContentText>
                </DialogContent>
                <DialogActions>
                    <Button onClick={() => setMoveConfirmOpen(false)}>キャンセル</Button>
                    <Button onClick={executeAutoMove} color="secondary" variant="contained" autoFocus>
                        移動開始
                    </Button>
                </DialogActions>
            </Dialog>
        </Box>
    )
}