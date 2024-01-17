import torch
import torch.nn as nn

class ActionLSTM(nn.Module):
    def __init__(self, input_len, hidden_size, num_layers, num_class, device):
        super(ActionLSTM, self).__init__()
        self.device = device
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_len, hidden_size, num_layers, batch_first=True)
        self.output_layer = nn.Linear(hidden_size, num_class)

    def forward(self, x):
        hidden_states = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to('cuda:0')
        cell_states = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to('cuda:0')
        out, _ = self.lstm(x, (hidden_states, cell_states))
        out = self.output_layer(out[:, -1, :])

        return out