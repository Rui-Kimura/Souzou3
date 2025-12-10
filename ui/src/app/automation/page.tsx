import { Box, Paper, Typography } from "@mui/material";
import Map from '@/components/Map'



export default function Page() {

    return(
    <Box sx={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <Paper sx={{ width: "80%", p: 2 }}>
        <Box>
            <Typography>自動移動</Typography>
            <Map/>
            </Box>
        </Paper>
    </Box>

    )
}
