"use client";

import { useEffect, useState } from "react";
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
  Grid, // 追加: レイアウト調整用
} from "@mui/material";

interface Point {
  name: string;
  x: number;
  y: number;
  angle: number;
}

export default function Map() {
  const TILE_SIZE = 20;
  const TILE_WIDTH = 50; // 50mm

  const [mapdata, setMapdata] = useState<string[]>([]);
  const [costmapData, setCostmapData] = useState<string[]>([]);
  const [savedPoints, setSavedPoints] = useState<Point[]>([]);
  
  const [position_x, setPositionX] = useState<number>(0);
  const [position_y, setPositionY] = useState<number>(0);
  const [position_angle, setPositionAngle] = useState<number>(0);

  const [target_x, setTargetX] = useState<number>(0);
  const [target_y, setTargetY] = useState<number>(0);
  const [target_angle, setTargetAngle] = useState<number>(0);

  // ポップオーバー・ダイアログ制御用
  const [anchorEl, setAnchorEl] = useState<HTMLElement | null>(null);
  const [selectedGrid, setSelectedGrid] = useState<{r: number, c: number} | null>(null);
  const [selectedSavedPoint, setSelectedSavedPoint] = useState<Point | null>(null);
  
  const [isNameDialogOpen, setNameDialogOpen] = useState(false);
  const [pointName, setPointName] = useState("");
  // --- 追加: 角度入力用State ---
  const [pointAngle, setPointAngle] = useState<string | number>(0);

  // 警告表示用State
  const [warningOpen, setWarningOpen] = useState(false);

  // --- データ取得系 ---

  const fetch_mapdata = async () => {
    try {
      const res = await fetch("/api/local/mapdata");
      const data = await res.json();
      setMapdata(data.mapdata);
    } catch (error) {
      console.error("Failed to fetch map data:", error);
    }
  };

  const fetch_costmapdata = async () => {
    try {
      const res = await fetch("/api/local/costmapdata");
      const data = await res.json();
      setCostmapData(data.costmapdata || []);
    } catch (error) {
      console.error("Failed to fetch costmap data:", error);
    }
  };

  const fetch_saved_points = async () => {
    try {
      const res = await fetch("/api/local/saved_target_points");
      const data = await res.json();
      const pointsList = Array.isArray(data) ? data : (data.points || []);
      setSavedPoints(pointsList);
    } catch (error) {
      console.error("Failed to fetch saved points:", error);
    }
  };

  const fetch_position = async () => {
    try {
      const res = await fetch("/api/local/position");
      const data = await res.json();
      setPositionX(data.x);
      setPositionY(data.y);
      setPositionAngle(data.angle);
    } catch (error) {
      console.error("Failed to fetch position data:", error);
    }
  };

  const fetch_target_point = async () => {
    try {
      const res = await fetch("/api/local/target_point");
      const data = await res.json();
      setTargetX(data.x);
      setTargetY(data.y);
      setTargetAngle(data.angle);
    } catch (error) {
      console.error("Failed to fetch target point data:", error);
    }
  };

  const interval_function = async () => {
    await fetch_position();
    await fetch_target_point();
    await fetch_saved_points(); 
  };

  useEffect(() => {
    fetch_mapdata();
    fetch_costmapdata();
    fetch_saved_points();
    const intervalId = setInterval(interval_function, 500);
    return () => clearInterval(intervalId);
  }, []);

  // --- ロジック系 ---

  const findPointAtGrid = (r: number, c: number) => {
    return savedPoints.find(p => {
      const pr = Math.floor(p.y / TILE_WIDTH);
      const pc = Math.floor(p.x / TILE_WIDTH);
      return pr === r && pc === c;
    });
  };

  const handleTileClick = (event: React.MouseEvent<HTMLElement>, rowIndex: number, colIndex: number, isWall: boolean, isCost: boolean) => {
    if (isWall) return;

    // コスト領域チェック
    if (isCost) {
      setWarningOpen(true);
      return;
    }

    const existingPoint = findPointAtGrid(rowIndex, colIndex);

    setAnchorEl(event.currentTarget);
    setSelectedGrid({ r: rowIndex, c: colIndex });
    
    if (existingPoint) {
      setSelectedSavedPoint(existingPoint);
      setPointName(existingPoint.name);
      // 既存ポイントの角度をセット
      setPointAngle(existingPoint.angle); 
    } else {
      setSelectedSavedPoint(null);
      setPointName("");
      // 新規の場合は0（または現在のロボットの向きなど）をセット
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

  // --- 追加: 即座にその場所へ移動する機能 ---
  const handleMoveHere = async () => {
    if (!selectedGrid) return;

    // タイルの中心座標を計算
    const physicalX = selectedGrid.c * TILE_WIDTH + TILE_WIDTH / 2.0;
    const physicalY = selectedGrid.r * TILE_WIDTH + TILE_WIDTH / 2.0;
    
    // 移動時の角度は登録ポイントがあればその角度、なければ0度とする（必要に応じて変更可）
    const angle = selectedSavedPoint ? selectedSavedPoint.angle : 0;

    const Point = {
      name:"",
      x: physicalX,
      y: physicalY,
      angle: angle
    };

    try {
      // ターゲット更新用のAPIエンドポイントへPOST (パスは環境に合わせて調整してください)
      const res = await fetch("/api/local/set_target_point", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(Point),
      });

      if (res.ok) {
        console.log("Target updated to click position");
        fetch_target_point(); // 反映のために即時取得
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

    // 名前変更の場合、古い名前のエントリを削除してから追加（上書き処理）
    if (selectedSavedPoint && selectedSavedPoint.name !== pointName) {
        await handleDeleteRequest(selectedSavedPoint.name);
    }

    const physicalX = selectedGrid.c * TILE_WIDTH + TILE_WIDTH / 2.0;
    const physicalY = selectedGrid.r * TILE_WIDTH + TILE_WIDTH / 2.0;
    
    // 入力された角度を使用
    const angle = Number(pointAngle);

    const payload: Point = {
      name: pointName || "No Name",
      x: physicalX,
      y: physicalY,
      angle: angle,
    };

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
        if (res.ok) {
            console.log("Deleted:", name);
        }
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

  // --- 描画用計算 ---
  const playerPixelX = (position_x / TILE_WIDTH) * TILE_SIZE;
  const playerPixelY = (position_y / TILE_WIDTH) * TILE_SIZE;
  const targetPixelX = (target_x / TILE_WIDTH) * TILE_SIZE;
  const targetPixelY = (target_y / TILE_WIDTH) * TILE_SIZE;

  const iconHalfWidth = TILE_SIZE * 4.5;
  const iconWidth = iconHalfWidth * 2;
  const iconHeight = TILE_SIZE * 0.8 * 4.5;

  const openPopover = Boolean(anchorEl);

  return (
    <Box
      sx={{
        minHeight: "100vh",
        bgcolor: "#f5f6fa",
        py: 4,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
      }}
    >
      <Container maxWidth="md" sx={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
        <Card elevation={3} sx={{ overflow: "hidden", maxWidth: "100%" }}>
          <Box
            sx={{
              position: "relative",
              padding: "10px",
              overflow: "auto",
              bgcolor: "#fff",
              display: "flex",
              flexDirection: "column",
            }}
          >
            {/* プレイヤーアイコン */}
            <Box
              sx={{
                position: "absolute",
                width: iconWidth,
                height: iconHeight,
                bgcolor: "#e74c3c",
                clipPath: "polygon(50% 0%, 0% 100%, 100% 100%)",
                left: playerPixelX,
                top: playerPixelY,
                transform: `translate(-50%, -50%) rotate(${position_angle}deg)`,
                transformOrigin: "center center", 
                zIndex: 20,
                transition: "all 0.3s ease-out",
                pointerEvents: "none",
              }}
            />

            {/* ターゲットアイコン */}
            <Box
              sx={{
                position: "absolute",
                width: iconWidth,
                height: iconHeight,
                bgcolor: "#4400ffff",
                clipPath: "polygon(50% 0%, 0% 100%, 100% 100%)",
                left: targetPixelX, 
                top: targetPixelY,
                transform: `translate(-50%, -50%) rotate(${target_angle}deg)`,
                transformOrigin: "center center",
                zIndex: 20,
                pointerEvents: "none",
              }}
            />
            
            {/* マップ描画 */}
            {mapdata.map((rowString: string, rowIndex) => (
              <Box key={rowIndex} sx={{ display: "flex" }}>
                {rowString.split("").map((cellChar, colIndex) => {
                  const isCost = costmapData[rowIndex]?.[colIndex] === "1";
                  const isWall = cellChar === "1";
                  const savedPoint = findPointAtGrid(rowIndex, colIndex);
                  
                  return (
                    <Box
                      key={`${rowIndex}-${colIndex}`}
                      onClick={(e) => handleTileClick(e, rowIndex, colIndex, isWall, isCost)}
                      sx={{
                        width: TILE_SIZE,
                        height: TILE_SIZE,
                        boxSizing: "border-box",
                        position: "relative",
                        bgcolor: isWall 
                            ? "#2c3e50" 
                            : savedPoint 
                                ? "#3498db" 
                                : "#ecf0f1",
                        border: cellChar === "0" ? "1px solid #bdc3c7" : "1px solid #34495e",
                        backgroundImage: (!isWall && !savedPoint && isCost)
                          ? "repeating-linear-gradient(45deg, rgba(255, 0, 0, 0.15) 0, rgba(255, 0, 0, 0.15) 2px, transparent 2px, transparent 6px)" 
                          : "none",
                        cursor: (!isWall && !isCost) ? "pointer" : (isCost ? "not-allowed" : "default"),
                        "&:hover": (!isWall && !isCost) ? { bgcolor: "#d6eaf8" } : {}
                      }}
                    >
                        {/* 保存されたポイントの矢印 */}
                        {savedPoint && (
                            <Box
                                sx={{
                                    position: "absolute",
                                    top: "50%",
                                    left: "50%",
                                    width: "60%",
                                    height: "60%",
                                    bgcolor: "white",
                                    clipPath: "polygon(50% 0%, 0% 100%, 50% 80%, 100% 100%)",
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
          </Box>
        </Card>

        {/* --- ポップオーバー --- */}
        <Popover
          open={openPopover}
          anchorEl={anchorEl}
          onClose={handlePopoverClose}
          anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
          transformOrigin={{ vertical: 'top', horizontal: 'center' }}
        >
          <Box sx={{ p: 2, display: 'flex', flexDirection: 'column', gap: 1, minWidth: 160 }}>
            {selectedSavedPoint ? (
                <>
                    <Typography variant="subtitle2" align="center" sx={{ fontWeight: 'bold' }}>
                        {selectedSavedPoint.name}
                    </Typography>
                    <Typography variant="caption" align="center" color="text.secondary">
                        {selectedSavedPoint.angle}°
                    </Typography>
                    <Button size="small" variant="contained" onClick={handleOpenDialog}>
                        編集
                    </Button>
                </>
            ) : (
                <Button size="small" variant="outlined" onClick={handleOpenDialog}>
                    地点として登録
                </Button>
            )}
            
            {/* --- 追加: ここに移動するボタン --- */}
            <Button size="small" variant="contained" color="secondary" onClick={handleMoveHere}>
                ここに移動する
            </Button>
          </Box>
        </Popover>

        {/* --- 名前・角度入力ダイアログ --- */}
        <Dialog open={isNameDialogOpen} onClose={() => setNameDialogOpen(false)}>
          <DialogTitle>
              {selectedSavedPoint ? "ポイントの編集" : "目標地点の追加"}
          </DialogTitle>
          <DialogContent>
            <Grid container spacing={2} sx={{ mt: 0.5 }}>
                <Grid item xs={8}>
                    <TextField
                    autoFocus
                    margin="dense"
                    label="地点名"
                    type="text"
                    fullWidth
                    variant="standard"
                    value={pointName}
                    onChange={(e) => setPointName(e.target.value)}
                    />
                </Grid>
                <Grid item xs={4}>
                    {/* --- 追加: 角度入力フォーム --- */}
                    <TextField
                    margin="dense"
                    label="角度 (deg)"
                    type="number"
                    fullWidth
                    variant="standard"
                    value={pointAngle}
                    onChange={(e) => setPointAngle(e.target.value)}
                    />
                </Grid>
            </Grid>
          </DialogContent>
          <DialogActions sx={{ justifyContent: 'space-between', px: 2, pb: 2 }}>
            {selectedSavedPoint ? (
                <Button onClick={handleDeletePoint} color="error">
                    削除
                </Button>
            ) : (
                <Box />
            )}
            <Box>
                <Button onClick={() => setNameDialogOpen(false)} sx={{ mr: 1 }}>
                    キャンセル
                </Button>
                <Button onClick={handleSavePoint} variant="contained">
                    保存
                </Button>
            </Box>
          </DialogActions>
        </Dialog>

        {/* 警告用 Snackbar */}
        <Snackbar
          open={warningOpen}
          autoHideDuration={1000}
          onClose={() => setWarningOpen(false)}
          message="周囲の構造物と干渉する恐れがあるため目標地点を設定できません。"
          anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
        />

        <Paper variant="outlined" sx={{ mt: 2, p: 1, px: 2, bgcolor: 'rgba(255,255,255,0.6)', borderColor: 'transparent' }}>
          <Typography variant="body2" color="text.secondary">
            現在位置: <strong>X={position_x.toFixed(0)}</strong>, <strong>Y={position_y.toFixed(0)}</strong>, 角度={position_angle.toFixed(0)}°
          </Typography>
        </Paper>

        <Stack direction="row" spacing={3} sx={{ mt: 3 }}>
          <Stack direction="row" alignItems="center" spacing={1}>
            <Box sx={{ width: 16, height: 16, bgcolor: "#2c3e50", borderRadius: 0.5 }} />
            <Typography variant="caption" color="text.secondary">壁</Typography>
          </Stack>
          <Stack direction="row" alignItems="center" spacing={1}>
            <Box sx={{ width: 16, height: 16, bgcolor: "#3498db", borderRadius: 0.5 }} />
            <Typography variant="caption" color="text.secondary">登録地点</Typography>
          </Stack>
          <Stack direction="row" alignItems="center" spacing={1}>
            <Box sx={{ width: 16, height: 16, bgcolor: "#ecf0f1", border: "1px solid #bdc3c7", borderRadius: 0.5 }} />
            <Typography variant="caption" color="text.secondary">通路</Typography>
          </Stack>
          <Stack direction="row" alignItems="center" spacing={1}>
            <Box sx={{ width: 16, height: 16, bgcolor: "#ecf0f1", border: "1px solid #bdc3c7", borderRadius: 0.5, backgroundImage: "repeating-linear-gradient(45deg, rgba(255, 0, 0, 0.25) 0, rgba(255, 0, 0, 0.25) 2px, transparent 2px, transparent 6px)" }} />
            <Typography variant="caption" color="text.secondary">コスト領域</Typography>
          </Stack>
        </Stack>
      </Container>
    </Box>
  );
}