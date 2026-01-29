"use client"
import React, { useState, useEffect } from 'react';
import { 
    Box, 
    Typography, 
    Card, 
    Button, 
    Dialog, 
    DialogTitle, 
    DialogContent, 
    DialogActions,
    TextField,
    Select,
    MenuItem,
    FormControl,
    InputLabel,
    Stack,
    IconButton,
    Container,
    Grid, // MUI v6では Grid2 が Grid としてエクスポートされている場合があります
    Chip,
    Alert
} from "@mui/material";
import AddIcon from '@mui/icons-material/Add';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
import PlaceIcon from '@mui/icons-material/Place';

// --- Types ---
interface Point {
    name: string,
    x: number;
    y: number;
    angle: number;
}

interface ScheduleItem {
    id: string;
    name: string;
    Day: string;
    time: string;
    position: string;
    tableId: string;
}

interface TableItem {
    id: string;
    name: string;
    memo: string;
}

const DAY_OPTIONS = [
    { value: "Monday", label: "月曜日" },
    { value: "Tuesday", label: "火曜日" },
    { value: "Wednesday", label: "水曜日" },
    { value: "Thursday", label: "木曜日" },
    { value: "Friday", label: "金曜日" },
    { value: "Saturday", label: "土曜日" },
    { value: "Sunday", label: "日曜日" },
];

export default function SchedulePage() {
    // --- State ---
    const [points, setPoints] = useState<Point[]>([]);
    const [schedules, setSchedules] = useState<ScheduleItem[]>([]);
    const [tables, setTables] = useState<TableItem[]>([]);
    const [isScheduleDialogOpen, setScheduleDialogOpen] = useState(false);
    const [editingSchedule, setEditingSchedule] = useState<ScheduleItem | null>(null);
    const [validationError, setValidationError] = useState<string | null>(null);

    // --- Data Fetching ---
    const fetchAllData = async () => {
        try {
            const [resPoints, resSchedules, resTables] = await Promise.all([
                fetch('/api/local/saved_target_points'),
                fetch('/api/local/get_schedules'),
                fetch('/api/local/table_data')
            ]);

            if (resPoints.ok) setPoints(await resPoints.json());
            if (resSchedules.ok) setSchedules(await resSchedules.json());
            if (resTables.ok) setTables(await resTables.json());
        } catch (error) {
            console.error('Error fetching data:', error);
        }
    };

    useEffect(() => {
        fetchAllData();
    }, []);

    // --- Handlers ---
    const handleOpenScheduleDialog = async (schedule?: ScheduleItem) => {
        await fetchAllData();
        setValidationError(null);

        if (schedule) {
            setEditingSchedule(schedule);
        } else {
            setEditingSchedule({
                id: crypto.randomUUID(),
                name: "",
                Day: "Monday",
                time: "12:00",
                position: "", 
                tableId: "" 
            });
        }
        setScheduleDialogOpen(true);
    };

    const handleSaveSchedule = async () => {
        if (!editingSchedule) return;

        if (
            !editingSchedule.name.trim() ||
            !editingSchedule.Day ||
            !editingSchedule.time ||
            !editingSchedule.position ||
            !editingSchedule.tableId
        ) {
            setValidationError("すべての項目を入力してください。");
            return;
        }

        try {
            const res = await fetch('/api/local/set_schedule', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(editingSchedule),
            });

            if (res.ok) {
                await fetchAllData();
                setScheduleDialogOpen(false);
                setEditingSchedule(null);
            } else {
                setValidationError("保存に失敗しました。");
            }
        } catch (e) {
            console.error(e);
            setValidationError("通信エラーが発生しました。");
        }
    };

    const handleDeleteSchedule = async (id: string) => {
        if (!confirm("このスケジュールを削除してもよろしいですか？")) return;
        try {
            const res = await fetch(`/api/local/delete_schedule?id=${id}`);
            if (res.ok) {
                await fetchAllData();
            }
        } catch (e) {
            console.error(e);
        }
    };

    const getTableNameById = (id: string) => {
        const table = tables.find(t => t.id === id);
        return table ? table.name : id;
    };

    // --- Render ---
    return (
        <Box sx={{ 
            width: "100%", minHeight: "100vh", 
            bgcolor: "#f5f5f5", p: 3, boxSizing: "border-box" 
        }}>
            <Container maxWidth="md">
                {/* Header */}
                <Box sx={{ mb: 4, display: "flex", alignItems: "center", justifyContent: "space-between" }}>
                    <Typography variant="h4" fontWeight="bold" color="text.primary">
                        スケジュール管理
                    </Typography>
                    <Button 
                        variant="contained" 
                        color="primary" 
                        startIcon={<AddIcon />}
                        onClick={() => handleOpenScheduleDialog()}
                        sx={{ borderRadius: 2, px: 3, py: 1, boxShadow: 2 }}
                    >
                        新規作成
                    </Button>
                </Box>

                {/* Schedule List */}
                <Grid container spacing={3}>
                    {schedules.map((sch) => {
                        const dayLabel = DAY_OPTIONS.find(d => d.value === sch.Day)?.label || sch.Day;
                        return (
                            /* 修正箇所: itemプロパティを削除し、size={12} を使用 */
                            <Grid size={12} key={sch.id}>
                                <Card sx={{ 
                                    display: 'flex', 
                                    flexDirection: 'column',
                                    position: 'relative',
                                    overflow: 'hidden',
                                    borderRadius: 2,
                                    boxShadow: 1,
                                    borderLeft: '6px solid #26a69a',
                                    transition: 'transform 0.2s, box-shadow 0.2s',
                                    '&:hover': {
                                        transform: 'translateY(-2px)',
                                        boxShadow: 3
                                    }
                                }}>
                                    <Box sx={{ p: 2, display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', gap: 2 }}>
                                        <Box sx={{ flexGrow: 1 }}>
                                            <Stack direction="row" spacing={1} alignItems="center" sx={{ mb: 1 }}>
                                                <Chip 
                                                    label={`ID: ${sch.id}`} 
                                                    size="small" 
                                                    variant="outlined" 
                                                    sx={{ 
                                                        borderColor: '#26a69a', 
                                                        color: '#26a69a', 
                                                        fontWeight: 'bold',
                                                        fontSize: '0.7rem',
                                                        height: 24
                                                    }} 
                                                />
                                            </Stack>

                                            <Typography variant="h5" fontWeight="bold" gutterBottom>
                                                {sch.name}
                                            </Typography>
                                            <Typography variant="subtitle1" color="text.secondary" sx={{ mb: 2 }}>
                                                {getTableNameById(sch.tableId)}
                                            </Typography>

                                            <Stack direction={{ xs: 'column', sm: 'row' }} spacing={{ xs: 1, sm: 3 }} color="text.secondary">
                                                <Box display="flex" alignItems="center" gap={0.5}>
                                                    <AccessTimeIcon fontSize="small" color="action" />
                                                    <Typography variant="body2" fontWeight="medium">
                                                        {dayLabel} {sch.time}
                                                    </Typography>
                                                </Box>
                                                <Box display="flex" alignItems="center" gap={0.5}>
                                                    <PlaceIcon fontSize="small" color="action" />
                                                    <Typography variant="body2" fontWeight="medium">
                                                        {sch.position}
                                                    </Typography>
                                                </Box>
                                            </Stack>
                                        </Box>

                                        {/* Actions */}
                                        <Stack direction="row" spacing={1}>
                                            <IconButton onClick={() => handleOpenScheduleDialog(sch)} color="default">
                                                <EditIcon />
                                            </IconButton>
                                            <IconButton onClick={() => handleDeleteSchedule(sch.id)} color="error">
                                                <DeleteIcon />
                                            </IconButton>
                                        </Stack>
                                    </Box>
                                </Card>
                            </Grid>
                        );
                    })}
                    {schedules.length === 0 && (
                        /* 修正箇所: itemプロパティを削除し、size={12} を使用 */
                        <Grid size={12}>
                            <Box sx={{ textAlign: 'center', py: 8, opacity: 0.6 }}>
                                <Typography variant="h6" color="text.secondary">
                                    スケジュールが登録されていません
                                </Typography>
                            </Box>
                        </Grid>
                    )}
                </Grid>
            </Container>

            {/* Schedule Edit/Create Dialog */}
            <Dialog open={isScheduleDialogOpen} onClose={() => setScheduleDialogOpen(false)} maxWidth="sm" fullWidth>
                <DialogTitle sx={{ borderBottom: '1px solid #eee', mb: 2 }}>
                    {editingSchedule?.name ? "スケジュール編集" : "新規スケジュール作成"}
                </DialogTitle>
                <DialogContent>
                    {/* Error Message */}
                    {validationError && (
                        <Alert severity="error" sx={{ mb: 2 }}>
                            {validationError}
                        </Alert>
                    )}

                    <Stack spacing={3}>
                        <TextField
                            label="スケジュール名"
                            fullWidth
                            required
                            placeholder="例: 朝のセット"
                            value={editingSchedule?.name || ""}
                            onChange={(e) => setEditingSchedule(prev => prev ? ({ ...prev, name: e.target.value }) : null)}
                        />

                        <Stack direction="row" spacing={2}>
                            <FormControl fullWidth required>
                                <InputLabel>曜日</InputLabel>
                                <Select
                                    value={editingSchedule?.Day || "Monday"}
                                    label="曜日"
                                    onChange={(e) => setEditingSchedule(prev => prev ? ({ ...prev, Day: e.target.value }) : null)}
                                >
                                    {DAY_OPTIONS.map(day => (
                                        <MenuItem key={day.value} value={day.value}>{day.label}</MenuItem>
                                    ))}
                                </Select>
                            </FormControl>

                            <TextField
                                label="実行時刻"
                                type="time"
                                fullWidth
                                required
                                InputLabelProps={{ shrink: true }}
                                value={editingSchedule?.time || ""}
                                onChange={(e) => setEditingSchedule(prev => prev ? ({ ...prev, time: e.target.value }) : null)}
                            />
                        </Stack>

                        <FormControl fullWidth required>
                            <InputLabel>移動先 (登録地点)</InputLabel>
                            <Select
                                value={editingSchedule?.position || ""}
                                label="移動先 (登録地点)"
                                onChange={(e) => setEditingSchedule(prev => prev ? ({ ...prev, position: e.target.value }) : null)}
                            >
                                {points.map(p => (
                                    <MenuItem key={p.name} value={p.name}>{p.name}</MenuItem>
                                ))}
                                {points.length === 0 && (
                                    <MenuItem disabled value=""><em>地点が登録されていません</em></MenuItem>
                                )}
                            </Select>
                        </FormControl>

                        <FormControl fullWidth required>
                            <InputLabel>対象テーブル (ID/名称)</InputLabel>
                            <Select
                                value={editingSchedule?.tableId || ""}
                                label="対象テーブル (ID/名称)"
                                onChange={(e) => setEditingSchedule(prev => prev ? ({ ...prev, tableId: e.target.value }) : null)}
                            >
                                {tables.map(table => (
                                    <MenuItem key={table.id} value={table.id}>{table.name}</MenuItem>
                                ))}
                                {tables.length === 0 && (
                                    <MenuItem disabled value=""><em>テーブルが登録されていません</em></MenuItem>
                                )}
                            </Select>
                        </FormControl>
                    </Stack>
                </DialogContent>
                <DialogActions sx={{ p: 3, pt: 0 }}>
                    <Button onClick={() => setScheduleDialogOpen(false)} sx={{ color: 'text.secondary' }}>キャンセル</Button>
                    <Button 
                        onClick={handleSaveSchedule} 
                        variant="contained" 
                        color="primary"
                        sx={{ px: 4, borderRadius: 2 }}
                    >
                        保存
                    </Button>
                </DialogActions>
            </Dialog>
        </Box>
    )
}