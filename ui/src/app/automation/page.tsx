import { Box, Paper, Typography } from "@mui/material";

export default function Home() {

    return(
    <Box sx={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <Paper sx={{ width: "80%", p: 2 }}>
        <Box>
            <Typography>自動移動</Typography>
            </Box>
        </Paper>
    </Box>

    )
}
