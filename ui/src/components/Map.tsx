"use client";

import { useEffect, useState, useRef } from "react";
import {
  Card,
  Box,
  Typography,
  Container,
  Stack,
  Paper,
  Popover,
  Button,
  Dialog,
  DialogTitle,
  DialogContent,
  TextField,
  DialogActions,
  Snackbar,
  Grid,
  Slider,
} from "@mui/material";

interface Point {
  name: string;
  x: number;
  y: number;
  angle: number;
}

interface StockerData {
  name: string;
  x: number;
  y: number;
  angle: number;
}

export default function Map() {
  const TILE_SIZE = 20;
  const TILE_WIDTH = 50; // 50mm

  // --- ストッカー形状定義 (180度反転: 前方配置) ---
  const GREEN_FRAME_POINTS = [
    // 奥のバー (Y: -400mm, 幅11マス)
    ...Array.from({ length: 11 }, (_, i) => ({ x: -250 + i * 50, y: -400 })),
    // 左アーム (X: -250mm, Y: -200 〜 -350mm)
    ...Array.from({ length: 4 }, (_, i) => ({ x: -250, y: -200 - i * 50 })),
    // 右アーム (X: 250mm, Y: -200 〜 -350mm)
    ...Array.from({ length: 4 }, (_, i) => ({ x: 250, y: -200 - i * 50 })),
  ];

  const [mapdata, setMapdata] = useState<string[]>([]);
  const [costmapData, setCostmapData] = useState<string[]>([]);
  const [savedPoints, setSavedPoints] = useState<Point[]>([]);
  
  const [existingStocker, setExistingStocker] = useState<StockerData | null>(null);

  const [position_x, setPositionX] = useState<number>(0);
  const [position_y, setPositionY] = useState<number>(0);
  const [position_angle, setPositionAngle] = useState<number>(0);

  const [target_x, setTargetX] = useState<number>(0);
  const [target_y, setTargetY] = useState<number>(0);
  const [target_angle, setTargetAngle] = useState<number>(0);

  const [anchorEl, setAnchorEl] = useState<HTMLElement | null>(null);
  const [selectedGrid, setSelectedGrid] = useState<{r: number, c: number} | null>(null);
  const [selectedSavedPoint, setSelectedSavedPoint] = useState<Point | null>(null);
  
  const [isNameDialogOpen, setNameDialogOpen] = useState(false);
  const [pointName, setPointName] = useState("");
  const [pointAngle, setPointAngle] = useState<string | number>(0);

  const [warningOpen, setWarningOpen] = useState(false);
  const [warningMessage, setWarningMessage] = useState(""); 
  const [snackMessage, setSnackMessage] = useState("");
  const [snackOpen, setSnackOpen] = useState(false);

  // ストッカー配置モード用State
  const [isStockerNameDialogOpen, setStockerNameDialogOpen] = useState(false);
  const [isPlacementMode, setPlacementMode] = useState(false);
  const [isPlacementFrozen, setPlacementFrozen] = useState(false);
  const [tempStocker, setTempStocker] = useState<StockerData>({ name: "", x: 0, y: 0, angle: 0 });
  const [stockerGridKeys, setStockerGridKeys] = useState<Set<string>>(new Set());
  const [placementValid, setPlacementValid] = useState(true);
  const [validationMsg, setValidationMsg] = useState("");

  const mapContainerRef = useRef<HTMLDivElement>(null);

  // --- WebSocket接続 ---
  useEffect(() => {
    const ws = new WebSocket("ws://localhost:8100/ws");

    ws.onopen = () => {
      console.log("Connected to WebSocket");
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.type === 'position') {
          setPositionX(data.x);
          setPositionY(data.y);
          setPositionAngle(data.angle);
        }
      } catch (e) {
        console.error("Failed to parse WS message", e);
      }
    };

    return () => {
      if (ws.readyState === WebSocket.OPEN) ws.close();
    };
  }, []);

  // --- データ取得 ---
  const fetch_mapdata = async () => {
    try {
      const res = await fetch("/api/local/mapdata");
      const data = await res.json();
      setMapdata(data.mapdata);
    } catch (error) { console.error(error); }
  };

  // 修正: mode引数を追加
  const fetch_costmapdata = async (mode = "normal") => {
    try {
      const res = await fetch(`/api/local/costmapdata?mode=${mode}`);
      const data = await res.json();
      setCostmapData(data.costmapdata || []);
    } catch (error) { console.error(error); }
  };

  const fetch_saved_points = async () => {
    try {
      const res = await fetch("/api/local/saved_target_points");
      const data = await res.json();
      setSavedPoints(Array.isArray(data) ? data : (data.points || []));
    } catch (error) { console.error(error); }
  };

  const fetch_target_point = async () => {
    try {
      const res = await fetch("/api/local/target_point");
      const data = await res.json();
      setTargetX(data.x);
      setTargetY(data.y);
      setTargetAngle(data.angle);
    } catch (error) { console.error(error); }
  };

  const fetch_stocker = async () => {
    try {
      const res = await fetch("/api/local/get_stocker");
      if (res.ok) {
        const data = await res.json();
        if (data && data.name) setExistingStocker(data);
        else setExistingStocker(null);
      }
    } catch (error) { console.error(error); }
  };

  useEffect(() => {
    fetch_mapdata();
    fetch_costmapdata("normal");
    fetch_saved_points();
    fetch_stocker();
  }, []);

  // 既存ストッカーの描画反映
  useEffect(() => {
    if (!isPlacementMode && existingStocker) {
      updateStockerGridKeys(existingStocker.x, existingStocker.y, existingStocker.angle);
    } else if (!isPlacementMode && !existingStocker) {
      setStockerGridKeys(new Set());
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isPlacementMode, existingStocker, mapdata]);


  // --- ロジック ---

  const findPointAtGrid = (r: number, c: number) => {
    return savedPoints.find(p => {
      const pr = Math.floor(p.y / TILE_WIDTH);
      const pc = Math.floor(p.x / TILE_WIDTH);
      return pr === r && pc === c;
    });
  };

  const isWall = (r: number, c: number) => {
    if (r < 0 || c < 0 || r >= mapdata.length || c >= (mapdata[0]?.length || 0)) return true;
    return mapdata[r][c] === "1";
  };

  const calculateOccupiedCells = (cx: number, cy: number, deg: number) => {
    const rad = (deg * Math.PI) / 180;
    const cos = Math.cos(rad);
    const sin = Math.sin(rad);
    const keys = new Set<string>();

    GREEN_FRAME_POINTS.forEach(p => {
      const absX = cx + (p.x * cos - p.y * sin);
      const absY = cy + (p.x * sin + p.y * cos);
      const c = Math.floor(absX / TILE_WIDTH);
      const r = Math.floor(absY / TILE_WIDTH);
      if (r >= 0 && c >= 0 && mapdata.length > 0 && r < mapdata.length && c < mapdata[0].length) {
        keys.add(`${r}-${c}`);
      }
    });
    return keys;
  };

  const updateStockerGridKeys = (x: number, y: number, angle: number) => {
    const keys = calculateOccupiedCells(x, y, angle);
    setStockerGridKeys(keys);
  };

  const checkAreaOverlap = (cx: number, cy: number, deg: number, points: {x:number, y:number}[]) => {
    const rad = (deg * Math.PI) / 180;
    const cos = Math.cos(rad), sin = Math.sin(rad);
    let walls = 0; let costs = 0; let out = 0;

    for (const p of points) {
      const absX = cx + (p.x * cos - p.y * sin);
      const absY = cy + (p.x * sin + p.y * cos);
      const c = Math.floor(absX / TILE_WIDTH);
      const r = Math.floor(absY / TILE_WIDTH);
      if (r < 0 || c < 0 || r >= mapdata.length || c >= (mapdata[0]?.length || 0)) { out++; continue; }
      if (mapdata[r][c] === "1") walls++;
      if (costmapData[r] && costmapData[r][c] === "1") costs++;
    }
    return { walls, costs, out };
  };

  const validatePlacement = (tx: number, ty: number, angle: number) => {
    // 青点(0,0): 壁NG, コストNG
    const blueRes = checkAreaOverlap(tx, ty, angle, [{x:0, y:0}]);
    if (blueRes.walls > 0 || blueRes.out > 0) return { valid: false, msg: "中心点が壁または範囲外です" };
    if (blueRes.costs > 0) return { valid: false, msg: "水色点はコストマップに進入できません" };

    // 緑枠: 壁NG (コストOK)
    const stockerRes = checkAreaOverlap(tx, ty, angle, GREEN_FRAME_POINTS);
    if (stockerRes.walls > 0 || stockerRes.out > 0) return { valid: false, msg: "ストッカーが壁と重なっています" };

    return { valid: true, msg: "" };
  };


  // --- イベントハンドラ ---

  const handleStartStockerRegistration = () => {
    setPointName(existingStocker ? existingStocker.name : "Stocker_1");
    setStockerNameDialogOpen(true);
  };

  const handleStockerNameConfirm = () => {
    setStockerNameDialogOpen(false);
    let initialX = existingStocker ? existingStocker.x : (mapdata[0]?.length || 10) * TILE_WIDTH / 2;
    let initialY = existingStocker ? existingStocker.y : mapdata.length * TILE_WIDTH / 2;
    const initialAngle = existingStocker ? existingStocker.angle : 0;
    
    setTempStocker({ name: pointName, x: initialX, y: initialY, angle: initialAngle });
    updateStockerGridKeys(initialX, initialY, initialAngle);
    
    setPlacementMode(true); 
    setPlacementFrozen(false);
    setSnackMessage("配置モード: マスに描画されます。クリックで仮固定。");
    setSnackOpen(true);

    fetch_costmapdata("base");
  };

const handlePlacementMove = (clientX: number, clientY: number) => {
    if (isPlacementFrozen || !mapContainerRef.current || !mapdata.length) return;
    
    const rect = mapContainerRef.current.getBoundingClientRect();
    const mouseCol = Math.floor((clientX - rect.left) / TILE_SIZE);
    const mouseRow = Math.floor((clientY - rect.top) / TILE_SIZE);
    
    const mousePhysX = mouseCol * TILE_WIDTH + TILE_WIDTH / 2.0;
    const mousePhysY = mouseRow * TILE_WIDTH + TILE_WIDTH / 2.0;

    const validationResult = validatePlacement(mousePhysX, mousePhysY, tempStocker.angle);

    setTempStocker(prev => ({ ...prev, x: mousePhysX, y: mousePhysY }));
    updateStockerGridKeys(mousePhysX, mousePhysY, tempStocker.angle);

    setPlacementValid(validationResult.valid);
    setValidationMsg(validationResult.msg);
  };

  const onMapMouseMove = (e: React.MouseEvent) => { if (isPlacementMode) handlePlacementMove(e.clientX, e.clientY); };
  const onMapTouchMove = (e: React.TouchEvent) => { if (isPlacementMode) handlePlacementMove(e.touches[0].clientX, e.touches[0].clientY); };

  const handleTileClick = (event: React.MouseEvent<HTMLElement>, rowIndex: number, colIndex: number, isWall: boolean, isCost: boolean) => {
    if (isPlacementMode) {
        setPlacementFrozen(prev => !prev);
        if (!isPlacementFrozen) handlePlacementMove(event.clientX, event.clientY);
        return;
    }

    if (isWall) return;

    if (stockerGridKeys.has(`${rowIndex}-${colIndex}`)) {
      return;
    }
    
    if (isCost) {
      setWarningMessage("周囲の構造物と干渉する恐れがあるため目標地点を設定できません。");
      setWarningOpen(true);
      return;
    }

    const existingPoint = findPointAtGrid(rowIndex, colIndex);

    setAnchorEl(event.currentTarget);
    setSelectedGrid({ r: rowIndex, c: colIndex });
    
    if (existingPoint) {
      setSelectedSavedPoint(existingPoint);
      setPointName(existingPoint.name);
      setPointAngle(existingPoint.angle); 
    } else {
      setSelectedSavedPoint(null);
      setPointName("");
      setPointAngle(0);
    }
  };

  const handlePopoverClose = () => {
    setAnchorEl(null);
    setSelectedGrid(null);
    setSelectedSavedPoint(null);
  };

  const handleOpenDialog = () => {
    setAnchorEl(null);
    setNameDialogOpen(true);
  };

  const handleMoveHere = async () => {
    if (!selectedGrid) return;
    const physicalX = selectedGrid.c * TILE_WIDTH + TILE_WIDTH / 2.0;
    const physicalY = selectedGrid.r * TILE_WIDTH + TILE_WIDTH / 2.0;
    const angle = selectedSavedPoint ? selectedSavedPoint.angle : 0;
    const Point = { name:"", x: physicalX, y: physicalY, angle: angle };

    try {
      const res = await fetch("/api/local/set_target_point", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(Point),
      });
      if (res.ok) {
        console.log("Target updated to click position");
        fetch_target_point();
      } else {
        console.error("Failed to update target");
      }
    } catch (error) {
      console.error("Error moving to point:", error);
    }
    handlePopoverClose();
  };

  const handleSavePoint = async () => {
    if (!selectedGrid) return;
    if (selectedSavedPoint && selectedSavedPoint.name !== pointName) {
        await handleDeleteRequest(selectedSavedPoint.name);
    }
    const physicalX = selectedGrid.c * TILE_WIDTH + TILE_WIDTH / 2.0;
    const physicalY = selectedGrid.r * TILE_WIDTH + TILE_WIDTH / 2.0;
    const angle = Number(pointAngle);
    const payload: Point = { name: pointName || "No Name", x: physicalX, y: physicalY, angle: angle };

    try {
      const res = await fetch("/api/local/add_target_point", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (res.ok) {
        console.log("Point saved");
        fetch_saved_points(); 
      } else {
        console.error("Failed to save point");
      }
    } catch (error) {
      console.error("Error saving:", error);
    }
    setNameDialogOpen(false);
    handlePopoverClose();
  };

  const handleDeleteRequest = async (name: string) => {
      try {
        const res = await fetch(`/api/local/delete_target_point?name=${encodeURIComponent(name)}`);
        if (res.ok) console.log("Deleted:", name);
      } catch (error) {
          console.error("Delete failed:", error);
      }
  };

  const handleDeletePoint = async () => {
      if (selectedSavedPoint) {
          await handleDeleteRequest(selectedSavedPoint.name);
          await fetch_saved_points();
          setNameDialogOpen(false);
          handlePopoverClose();
      }
  };

  const rotateStocker = (deg: number) => {
    setTempStocker(prev => {
        const newAngle = (prev.angle + deg) % 360;
        updateStockerGridKeys(prev.x, prev.y, newAngle);
        const res = validatePlacement(prev.x, prev.y, newAngle);
        setPlacementValid(res.valid); setValidationMsg(res.msg);
        return { ...prev, angle: newAngle };
    });
  };

  const handleCommitStocker = async () => {
    if (!placementValid) return;
    try {
      const res = await fetch("/api/local/set_stocker", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(tempStocker) });
      if (res.ok) {
        setSnackMessage("ストッカーを登録しました"); setSnackOpen(true);
        fetch_stocker(); 
        setPlacementMode(false); 
        setPlacementFrozen(false);
        // ★保存完了したら通常マップに戻す
        fetch_costmapdata("normal");
      }
    } catch (error) { console.error(error); }
  };

  const playerPixelX = (position_x / TILE_WIDTH) * TILE_SIZE;
  const playerPixelY = (position_y / TILE_WIDTH) * TILE_SIZE;
  const targetPixelX = (target_x / TILE_WIDTH) * TILE_SIZE;
  const targetPixelY = (target_y / TILE_WIDTH) * TILE_SIZE;
  const iconSize = TILE_SIZE * 2.5; 
  const openPopover = Boolean(anchorEl);

  return (
    <Box sx={{ minHeight: "100vh", bgcolor: "#f5f6fa", py: 4, display: "flex", flexDirection: "column", alignItems: "center" }}>
      <Container maxWidth="md" sx={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
        
        <Box sx={{ width: '100%', display: 'flex', justifyContent: 'flex-end', mb: 2 }}>
            <Button variant="contained" color="success" onClick={handleStartStockerRegistration} disabled={isPlacementMode}>
                {existingStocker ? "ストッカー編集" : "ストッカー登録"}
            </Button>
        </Box>

        <Card elevation={3} sx={{ overflow: "hidden", maxWidth: "100%" }}>
          <Box
            ref={mapContainerRef}
            onMouseMove={onMapMouseMove}
            onTouchMove={onMapTouchMove}
            sx={{
              position: "relative", padding: "10px", overflow: "auto", bgcolor: "#fff", display: "flex", flexDirection: "column",
              touchAction: isPlacementMode ? "none" : "auto", 
              cursor: isPlacementMode ? (isPlacementFrozen ? "default" : "move") : "default"
            }}
          >
            {/* プレイヤーアイコン */}
            <Box
              sx={{
                position: "absolute", width: 0, height: 0,
                borderLeft: `${iconSize / 2}px solid transparent`,
                borderRight: `${iconSize / 2}px solid transparent`,
                borderBottom: `${iconSize}px solid #e74c3c`,
                left: playerPixelX + 10, top: playerPixelY + 10,
                transform: `translate(-50%, -50%) rotate(${position_angle}deg)`,
                zIndex: 20, pointerEvents: "none", transition: "all 0.1s ease-out"
              }}
            />

            {/* ターゲットアイコン */}
            {/*}
            <Box
              sx={{
                position: "absolute", width: 0, height: 0,
                borderLeft: `${iconSize / 2}px solid transparent`,
                borderRight: `${iconSize / 2}px solid transparent`,
                borderBottom: `${iconSize}px solid #3498db`,
                left: targetPixelX + 10, top: targetPixelY + 10,
                transform: `translate(-50%, -50%) rotate(${target_angle}deg)`,
                zIndex: 19, pointerEvents: "none", opacity: 0.8
              }}
            />*/}
            
            {/* マップ描画 */}
            {mapdata.map((rowString: string, rowIndex) => (
              <Box key={rowIndex} sx={{ display: "flex" }}>
                {rowString.split("").map((cellChar, colIndex) => {
                  const isCost = costmapData[rowIndex]?.[colIndex] === "1";
                  const isWall = cellChar === "1";
                  const savedPoint = findPointAtGrid(rowIndex, colIndex);
                  
                  const isStockerCell = stockerGridKeys.has(`${rowIndex}-${colIndex}`);
                  
                  let bgcolor = isWall ? "#2c3e50" : savedPoint ? "#3498db" : "#ecf0f1";
                  
                  if (!isWall && isStockerCell) {
                      if (isPlacementMode && !placementValid) bgcolor = "rgba(231, 76, 60, 0.7)"; 
                      else bgcolor = "#7be37b"; 
                  }

                  let bgImage = (!isWall && !savedPoint && isCost)
                          ? "repeating-linear-gradient(45deg, rgba(255, 0, 0, 0.15) 0, rgba(255, 0, 0, 0.15) 2px, transparent 2px, transparent 6px)" 
                          : "none";
                  
                  let border = cellChar === "0" ? "1px solid #bdc3c7" : "1px solid #34495e";
                  if (isStockerCell && isPlacementFrozen) border = "2px solid #2196f3";

                  return (
                    <Box
                      key={`${rowIndex}-${colIndex}`}
                      onClick={(e) => handleTileClick(e, rowIndex, colIndex, isWall, isCost)}
                      sx={{
                        width: TILE_SIZE, height: TILE_SIZE, boxSizing: "border-box", position: "relative",
                        bgcolor: bgcolor, border: border, backgroundImage: bgImage,
                        cursor: (!isWall && !isCost && !isStockerCell) ? "pointer" : (isCost ? "not-allowed" : "default"),
                        "&:hover": (!isWall && !isCost && !isPlacementMode) ? { bgcolor: "#d6eaf8" } : {}
                      }}
                    >
                        {savedPoint && (
                            <Box
                                sx={{
                                    position: "absolute", top: "50%", left: "50%", width: "60%", height: "60%",
                                    bgcolor: "white", clipPath: "polygon(50% 0%, 0% 100%, 50% 80%, 100% 100%)",
                                    transform: `translate(-50%, -50%) rotate(${savedPoint.angle}deg)`,
                                    transformOrigin: "center center",
                                }}
                            />
                        )}
                    </Box>
                  );
                })}
              </Box>
            ))}

            {/* 配置モード中の中心点マーカー */}
            {isPlacementMode && (
                 <Box
                    sx={{
                        position: "absolute",
                        left: (tempStocker.x / TILE_WIDTH) * TILE_SIZE,
                        top: (tempStocker.y / TILE_WIDTH) * TILE_SIZE,
                        width: 0, height: 0,
                        pointerEvents: "none", zIndex: 100,
                    }}
                 >
                     <Box sx={{ position:"absolute", width:14, height:14, bgcolor:"#03a9f4", transform:"translate(-50%,-50%)", zIndex:102, borderRadius: "50%" }} />
                 </Box>
             )}

          </Box>
        </Card>

        <Popover
          open={openPopover} anchorEl={anchorEl} onClose={handlePopoverClose}
          anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }} transformOrigin={{ vertical: 'top', horizontal: 'center' }}
        >
          <Box sx={{ p: 2, display: 'flex', flexDirection: 'column', gap: 1, minWidth: 160 }}>
            {selectedSavedPoint ? (
                <>
                    <Typography variant="subtitle2" align="center" sx={{ fontWeight: 'bold' }}>{selectedSavedPoint.name}</Typography>
                    <Typography variant="caption" align="center" color="text.secondary">{selectedSavedPoint.angle}°</Typography>
                    <Button size="small" variant="contained" onClick={handleOpenDialog}>編集</Button>
                </>
            ) : (
                <Button size="small" variant="outlined" onClick={handleOpenDialog}>地点として登録</Button>
            )}
            <Button size="small" variant="contained" color="secondary" onClick={handleMoveHere}>ここに移動する</Button>
          </Box>
        </Popover>

        <Dialog open={isNameDialogOpen} onClose={() => setNameDialogOpen(false)}>
          <DialogTitle>{selectedSavedPoint ? "ポイントの編集" : "目標地点の追加"}</DialogTitle>
          <DialogContent>
            <Box sx={{ display: 'flex', gap: 2, mt: 0.5 }}>
                <TextField autoFocus margin="dense" label="地点名" type="text" fullWidth variant="standard" value={pointName} onChange={(e) => setPointName(e.target.value)} sx={{ flex: 2 }} />
                <TextField margin="dense" label="角度 (deg)" type="number" fullWidth variant="standard" value={pointAngle} onChange={(e) => setPointAngle(e.target.value)} sx={{ flex: 1 }} />
            </Box>
          </DialogContent>
          <DialogActions sx={{ justifyContent: 'space-between', px: 2, pb: 2 }}>
            {selectedSavedPoint ? <Button onClick={handleDeletePoint} color="error">削除</Button> : <Box />}
            <Box>
                <Button onClick={() => setNameDialogOpen(false)} sx={{ mr: 1 }}>キャンセル</Button>
                <Button onClick={handleSavePoint} variant="contained">保存</Button>
            </Box>
          </DialogActions>
        </Dialog>

        <Dialog open={isStockerNameDialogOpen} onClose={()=>setStockerNameDialogOpen(false)}>
            <DialogTitle>ストッカー設定</DialogTitle>
            <DialogContent><TextField autoFocus margin="dense" label="名称" fullWidth value={pointName} onChange={(e)=>setPointName(e.target.value)} variant="standard" /></DialogContent>
            <DialogActions>
                <Button onClick={()=>setStockerNameDialogOpen(false)}>キャンセル</Button>
                <Button onClick={handleStockerNameConfirm} variant="contained">配置を開始</Button>
            </DialogActions>
        </Dialog>

        {isPlacementMode && (
          <Paper elevation={6} sx={{ position:'fixed', bottom:20, left:'50%', transform:'translateX(-50%)', width:'90%', maxWidth:600, p:2, zIndex:1300 }}>
             <Typography variant="subtitle2" align="center" sx={{mb:1, fontWeight:'bold', color: placementValid ? "success.main" : "error.main"}}>
                {isPlacementFrozen ? (placementValid ? "決定可能です" : "配置できません") : (placementValid ? (validationMsg || "配置中") : `⚠ ${validationMsg}`)}
             </Typography>
             <Box sx={{ display: 'flex', alignItems: 'center', gap: 2, mb: 2 }}>
                <Typography variant="caption">回転</Typography>
                <Slider value={tempStocker.angle} min={0} max={360} onChange={(_, v) => rotateStocker(v as number - tempStocker.angle)} sx={{ flexGrow: 1 }} />
                <Button variant="outlined" size="small" onClick={() => rotateStocker(90)}>+90°</Button>
             </Box>
             <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                <Button variant="outlined" color="error" onClick={()=>{
                    setPlacementMode(false); setPlacementFrozen(false);
                    updateStockerGridKeys(existingStocker?.x ?? 0, existingStocker?.y ?? 0, existingStocker?.angle ?? 0);
                    if(!existingStocker) setStockerGridKeys(new Set());
                    fetch_costmapdata("normal");
                }}>キャンセル</Button>
                <Button variant="contained" color="primary" onClick={handleCommitStocker} disabled={!placementValid || !isPlacementFrozen}>決定</Button>
             </Box>
          </Paper>
        )}

        <Snackbar open={warningOpen} autoHideDuration={1000} onClose={() => setWarningOpen(false)} message={warningMessage} anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }} />
        <Snackbar open={snackOpen} autoHideDuration={2000} onClose={() => setSnackOpen(false)} message={snackMessage} anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }} />

        <Paper variant="outlined" sx={{ mt: 2, p: 1, px: 2, bgcolor: 'rgba(255,255,255,0.6)', borderColor: 'transparent' }}>
          <Typography variant="body2" color="text.secondary">
            現在位置: <strong>X={position_x.toFixed(0)}</strong>, <strong>Y={position_y.toFixed(0)}</strong>, 角度={position_angle.toFixed(0)}°
          </Typography>
        </Paper>

        <Stack direction="row" spacing={3} sx={{ mt: 3 }}>
          <Stack direction="row" alignItems="center" spacing={1}><Box sx={{ width: 16, height: 16, bgcolor: "#2c3e50", borderRadius: 0.5 }} /><Typography variant="caption" color="text.secondary">壁</Typography></Stack>
          <Stack direction="row" alignItems="center" spacing={1}><Box sx={{ width: 16, height: 16, bgcolor: "#3498db", borderRadius: 0.5 }} /><Typography variant="caption" color="text.secondary">登録地点</Typography></Stack>
          <Stack direction="row" alignItems="center" spacing={1}><Box sx={{ width: 16, height: 16, bgcolor: "#ecf0f1", border: "1px solid #bdc3c7", borderRadius: 0.5 }} /><Typography variant="caption" color="text.secondary">通路</Typography></Stack>
          <Stack direction="row" alignItems="center" spacing={1}><Box sx={{ width: 16, height: 16, bgcolor: "#ecf0f1", border: "1px solid #bdc3c7", borderRadius: 0.5, backgroundImage: "repeating-linear-gradient(45deg, rgba(255, 0, 0, 0.25) 0, rgba(255, 0, 0, 0.25) 2px, transparent 2px, transparent 6px)" }} /><Typography variant="caption" color="text.secondary">コスト領域</Typography></Stack>
        </Stack>
      </Container>
    </Box>
  );
}