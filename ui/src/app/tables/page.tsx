"use client";

import { useEffect, useState } from "react";
import {
  Box,
  Container,
  Typography,
  Grid,
  Card,
  CardContent,
  CardActions,
  Button,
  CircularProgress,
  Alert,
  Chip,
  Stack,
  Snackbar,
  Collapse,
  IconButton,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  InputAdornment,
} from "@mui/material";
import { alpha } from "@mui/material/styles";
import AddIcon from "@mui/icons-material/Add";
import EditIcon from "@mui/icons-material/Edit";
import DeleteIcon from "@mui/icons-material/Delete";
import QrCodeScannerIcon from "@mui/icons-material/QrCodeScanner";
import BarcodeScannerComponent from "react-qr-barcode-scanner";

interface TableItem {
  id: string;
  name: string;
  memo: string;
}

export default function TablePage() {
  const [items, setItems] = useState<TableItem[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState("");
  const [selectedId, setSelectedId] = useState<string | null>(null);

  const [snackOpen, setSnackOpen] = useState(false);
  const [snackMessage, setSnackMessage] = useState("");

  const [dialogOpen, setDialogOpen] = useState(false);
  const [isEdit, setIsEdit] = useState(false);
  const [isScanning, setIsScanning] = useState(false);
  const [formData, setFormData] = useState<TableItem>({ id: "", name: "", memo: "" });

  const fetchData = async () => {
    try {
      setLoading(true);
      const res = await fetch("/api/local/table_data");
      if (!res.ok) throw new Error(`HTTP error! status: ${res.status}`);
      const data = await res.json();
      setItems(data);
    } catch (err) {
      console.error("Fetch error:", err);
      setError("データの取得に失敗しました。");
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchData();
  }, []);

  const handleCardClick = (id: string) => {
    setSelectedId((prev) => (prev === id ? null : id));
  };

  const handleDelete = async (e: React.MouseEvent, id: string) => {
    e.stopPropagation();
    if (!confirm("このデータを削除してもよろしいですか？")) return;

    try {
      const res = await fetch(`/api/local/delete_table_data?id=${id}`, { method: "GET" });
      const result = await res.json();
      if (res.ok && result.message === "deleted") {
        setSnackMessage("削除しました");
        fetchData();
      } else {
        setSnackMessage("削除に失敗しました");
      }
    } catch (err) {
      setSnackMessage("エラーが発生しました");
    } finally {
      setSnackOpen(true);
    }
  };

  const handleSave = async () => {
    if (!formData.id || !formData.name) {
      alert("IDと名称は必須です");
      return;
    }

    try {
      const res = await fetch("/api/local/add_table_data", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(formData),
      });

      if (res.ok) {
        setSnackMessage(isEdit ? "更新しました" : "追加しました");
        setDialogOpen(false);
        fetchData();
      } else {
        setSnackMessage("保存に失敗しました");
      }
    } catch (err) {
      setSnackMessage("エラーが発生しました");
    } finally {
      setSnackOpen(true);
    }
  };

  const openAddDialog = () => {
    setIsEdit(false);
    setIsScanning(false);
    setFormData({ id: "", name: "", memo: "" });
    setDialogOpen(true);
  };

  const openEditDialog = (e: React.MouseEvent, item: TableItem) => {
    e.stopPropagation();
    setIsEdit(true);
    setIsScanning(false);
    setFormData(item);
    setDialogOpen(true);
  };

  const handlePick = async (e: React.MouseEvent, id: string) => {
    e.stopPropagation();
    try {
      const res = await fetch(`/api/local/pick_table?id=${encodeURIComponent(id)}`);
      if (res.ok) setSnackMessage(`ID: ${id} の取り出し指示を送信しました`);
      else setSnackMessage("送信に失敗しました");
    } catch (error) {
      setSnackMessage("通信エラーが発生しました");
    } finally {
      setSnackOpen(true);
    }
  };

  return (
    <Box sx={{ width: "100%", minHeight: "100vh", bgcolor: "#f8f9fa", py: 4 }}>
      <Container maxWidth="lg">
        <Stack direction="row" justifyContent="space-between" alignItems="center" sx={{ mb: 4 }}>
          <Typography variant="h4" sx={{ fontWeight: "bold", color: "#333" }}>
            テーブル管理
          </Typography>
          <Button
            variant="contained"
            startIcon={<AddIcon />}
            onClick={openAddDialog}
            sx={{
              bgcolor: "#4db6ac",
              "&:hover": { bgcolor: "#3d9189" },
              borderRadius: 2,
              px: 3,
              fontWeight: "bold",
            }}
          >
            新規作成
          </Button>
        </Stack>

        {loading ? (
          <Box sx={{ display: "flex", justifyContent: "center", mt: 4 }}>
            <CircularProgress color="primary" />
          </Box>
        ) : error ? (
          <Alert severity="error">{error}</Alert>
        ) : (
          <Grid container spacing={3}>
            {items.map((item) => {
              const isSelected = selectedId === item.id;
              return (
                <Grid size={12} key={item.id}>
                  <Card
                    elevation={isSelected ? 4 : 1}
                    onClick={() => handleCardClick(item.id)}
                    sx={{
                      position: "relative",
                      borderRadius: 2,
                      borderLeft: 8,
                      borderColor: "#4db6ac",
                      transition: "0.2s",
                      cursor: "pointer",
                      "&:hover": { transform: "translateY(-2px)", boxShadow: 3 },
                    }}
                  >
                    <CardContent>
                      <Stack direction="row" justifyContent="space-between" alignItems="flex-start">
                        <Box>
                          <Chip
                            label={`ID: ${item.id}`}
                            size="small"
                            sx={{
                              mb: 1,
                              bgcolor: alpha("#4db6ac", 0.1),
                              color: "#00897b",
                              fontWeight: "bold",
                              border: "1px solid",
                              borderColor: "#4db6ac",
                            }}
                          />
                          <Typography variant="h5" sx={{ fontWeight: "bold", mb: 1 }}>
                            {item.name}
                          </Typography>
                          <Typography variant="body1" color="text.secondary">
                            {item.memo}
                          </Typography>
                        </Box>
                        <Stack direction="row" spacing={1}>
                          <IconButton size="small" onClick={(e) => openEditDialog(e, item)}>
                            <EditIcon fontSize="small" />
                          </IconButton>
                          <IconButton size="small" color="error" onClick={(e) => handleDelete(e, item.id)}>
                            <DeleteIcon fontSize="small" />
                          </IconButton>
                        </Stack>
                      </Stack>
                    </CardContent>

                    <Collapse in={isSelected} timeout="auto" unmountOnExit>
                      <CardActions sx={{ p: 2, pt: 0, bgcolor: alpha("#4db6ac", 0.02) }}>
                        <Button
                          variant="contained"
                          fullWidth
                          color="secondary"
                          onClick={(e) => handlePick(e, item.id)}
                          sx={{ fontWeight: "bold" }}
                        >
                          このテーブルを取り出す
                        </Button>
                      </CardActions>
                    </Collapse>
                  </Card>
                </Grid>
              );
            })}
          </Grid>
        )}
      </Container>

      <Dialog open={dialogOpen} onClose={() => setDialogOpen(false)} fullWidth maxWidth="xs">
        <DialogTitle sx={{ fontWeight: "bold" }}>
          {isEdit ? "テーブルの編集" : "テーブルの新規登録"}
        </DialogTitle>
        <DialogContent dividers>
          <Stack spacing={3} sx={{ mt: 1 }}>
            <TextField
              label="ID (JANコード)"
              fullWidth
              disabled={isEdit}
              value={formData.id}
              onChange={(e) => setFormData({ ...formData, id: e.target.value })}
              InputProps={{
                endAdornment: !isEdit && (
                  <InputAdornment position="end">
                    <IconButton color="primary" onClick={() => setIsScanning(!isScanning)} edge="end">
                      <QrCodeScannerIcon />
                    </IconButton>
                  </InputAdornment>
                ),
              }}
            />

            {isScanning && (
              <Box sx={{ width: "100%", overflow: "hidden", borderRadius: 2, bgcolor: "#000", position: "relative" }}>
                <BarcodeScannerComponent
                  width="100%"
                  onUpdate={(err, result) => {
                    if (result) {
                      setFormData({ ...formData, id: result.getText() });
                      setIsScanning(false);
                      setSnackMessage("バーコードを読み取りました");
                      setSnackOpen(true);
                    }
                  }}
                />
                <Button 
                  fullWidth 
                  variant="contained" 
                  color="error" 
                  onClick={() => setIsScanning(false)}
                  sx={{ borderRadius: 0 }}
                >
                  キャンセル
                </Button>
              </Box>
            )}

            <TextField
              label="名称"
              fullWidth
              value={formData.name}
              onChange={(e) => setFormData({ ...formData, name: e.target.value })}
            />
            <TextField
              label="メモ"
              fullWidth
              multiline
              rows={3}
              value={formData.memo}
              onChange={(e) => setFormData({ ...formData, memo: e.target.value })}
            />
          </Stack>
        </DialogContent>
        <DialogActions sx={{ p: 2 }}>
          <Button onClick={() => setDialogOpen(false)} color="inherit">
            キャンセル
          </Button>
          <Button onClick={handleSave} variant="contained" sx={{ bgcolor: "#4db6ac" }}>
            保存する
          </Button>
        </DialogActions>
      </Dialog>

      <Snackbar
        open={snackOpen}
        autoHideDuration={3000}
        onClose={() => setSnackOpen(false)}
        message={snackMessage}
        anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
      />
    </Box>
  );
}