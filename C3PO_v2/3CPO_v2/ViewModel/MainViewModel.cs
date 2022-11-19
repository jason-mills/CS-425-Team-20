using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace _3CPO_v2.ViewModel;

public partial class MainViewModel : ObservableObject
{
    [RelayCommand]
    async Task SettingsTap()
    {
        await Shell.Current.GoToAsync(nameof(SettingsPage));
    }
}
