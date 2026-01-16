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
  Collapse
} from "@mui/material";
import { alpha } from "@mui/material/styles";

interface TableItem {
  id: string;
  name: string;
  memo: string;
}

export default function TablePage() {
  const [items, setItems] = useState<TableItem[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState("");
  
  // 選択されたカードのIDを管理
  const [selectedId, setSelectedId] = useState<string | null>(null);
  
  // スナックバー（通知）用
  const [snackOpen, setSnackOpen] = useState(false);
  const [snackMessage, setSnackMessage] = useState("");

  useEffect(() => {
    const fetchData = async () => {
      try {
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
    fetchData();
  }, []);

  const handleCardClick = (id: string) => {
    setSelectedId(prev => prev === id ? null : id);
  };

  // 【修正箇所】GETリクエストに変更し、クエリパラメータでIDを送信
  const handlePick = async (e: React.MouseEvent, id: string) => {
    e.stopPropagation();

    try {
      // URLにクエリパラメータ ?id=... を付与してGETリクエスト
      const res = await fetch(`/api/local/pick_table?id=${encodeURIComponent(id)}`, {
        method: "GET",
      });

      if (res.ok) {
        setSnackMessage(`ID: ${id} の取り出し指示を送信しました`);
      } else {
        setSnackMessage("送信に失敗しました");
      }
    } catch (error) {
      console.error(error);
      setSnackMessage("通信エラーが発生しました");
    } finally {
      setSnackOpen(true);
    }
  };

  return (
    <Box
      sx={{
        width: "100%",
        minHeight: "100vh",
        bgcolor: "background.default",
        py: 4,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
      }}
    >
      <Container maxWidth="lg">
        <Typography 
          variant="h5" 
          component="h1" 
          sx={{ mb: 4, color: "text.primary", fontWeight: 'bold', textAlign: "center" }}
        >
          データ一覧
        </Typography>

        {loading ? (
          <Box sx={{ display: "flex", justifyContent: "center", mt: 4 }}>
            <CircularProgress color="primary" />
          </Box>
        ) : error ? (
          <Alert severity="error" sx={{ mt: 2 }}>{error}</Alert>
        ) : (
          <Grid container spacing={3}>
            {items.map((item) => {
              const isSelected = selectedId === item.id;
              
              return (
                <Grid key={item.id} size={{ xs: 12, sm: 6, md: 4 }}>
                  <Card
                    elevation={isSelected ? 8 : 2}
                    onClick={() => handleCardClick(item.id)}
                    sx={{
                      height: "100%",
                      display: "flex",
                      flexDirection: "column",
                      bgcolor: isSelected ? (theme) => alpha(theme.palette.primary.main, 0.05) : "background.paper",
                      transition: "all 0.3s ease",
                      borderLeft: 6,
                      borderColor: isSelected ? "secondary.main" : "primary.main",
                      cursor: "pointer",
                      transform: isSelected ? "scale(1.02)" : "scale(1)",
                      "&:hover": {
                        transform: isSelected ? "scale(1.02)" : "translateY(-4px)",
                        boxShadow: (theme) => `0 8px 24px ${alpha(theme.palette.primary.main, 0.15)}`,
                      },
                    }}
                  >
                    <CardContent sx={{ flexGrow: 1 }}>
                      <Stack direction="row" justifyContent="space-between" alignItems="flex-start" sx={{ mb: 2 }}>
                        <Chip 
                          label={`ID: ${item.id}`} 
                          size="small" 
                          variant={isSelected ? "filled" : "outlined"}
                          color="primary"
                          sx={{ fontWeight: "bold" }}
                        />
                      </Stack>
                      
                      <Typography gutterBottom variant="h6" component="div" sx={{ fontWeight: "bold", color: "text.primary" }}>
                        {item.name}
                      </Typography>
                      
                      <Typography variant="body2" color="text.secondary" sx={{ mt: 1, whiteSpace: "pre-wrap" }}>
                        {item.memo}
                      </Typography>
                    </CardContent>

                    <Collapse in={isSelected} timeout="auto" unmountOnExit>
                      <CardActions sx={{ p: 2, pt: 0, justifyContent: "flex-end" }}>
                        <Button 
                          variant="contained" 
                          color="secondary"
                          onClick={(e) => handlePick(e, item.id)}
                          fullWidth
                          sx={{ fontWeight: "bold" }}
                        >
                          取り出し
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